#include "feixi_ros2_control/feixi_mujoco_hardware.hpp"

#include "feixi_ros2_control/mj_view.hpp"

#include <cctype>
#include <cmath>
#include <filesystem>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#pragma GCC diagnostic pop
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace feixi_ros2_control {

namespace {

std::optional<double> trimmed_stod(char const* s) {
    //字符串转换为double
    if (!s || *s == '\0') return std::nullopt;
    try {
        return std::stod(std::string(s));
    } catch (...) {
        return std::nullopt;
    }
}

bool parse_csv_doubles8(std::string const& s, std::array<double, 8>* out) {
    if (!out) {
        return false;
    }
    std::string token;
    std::vector<double> vals;
    token.reserve(32);
    auto flush_token = [&]() {
        if (token.empty()) {
            return;
        }
        try {
            vals.push_back(std::stod(token));
        } catch (...) {
            vals.clear();
            token.clear();
            return;
        }
        token.clear();
    };
    for (unsigned char const uc : s) {
        char const c = static_cast<char>(uc);
        if (c == ',' || c == ';') {
            flush_token();
            continue;
        }
        if (std::isspace(static_cast<unsigned char>(c)) != 0) {
            flush_token();
            continue;
        }
        token.push_back(c);
    }
    flush_token();
    if (vals.size() != 8) {
        return false;
    }
    for (std::size_t i = 0; i < 8; ++i) {
        (*out)[i] = vals[i];
    }
    return true;
}

std::array<double, 8> default_init_pose_q() {
    // Upright-ish safe start (radians); override via hardware param init_pose_q.
    return {0.0, -0.5, 0.0, 1.2, 0.0, 0.8, 0.0, 0.05};
}

MujocoJointActuationMode parse_joint_actuation_mode(std::unordered_map<std::string, std::string> const& pmap) {
    auto const it = pmap.find("mujoco_joint_actuation");
    if (it == pmap.end() || it->second.empty()) {
        return MujocoJointActuationMode::PdTorque;
    }
    std::string v;
    v.reserve(it->second.size());
    for (unsigned char uc : it->second) {
        if (std::isspace(uc) == 0) {
            v.push_back(static_cast<char>(std::tolower(uc)));
        }
    }
    if (v == "direct" || v == "kinematic" || v == "qpos") {
        return MujocoJointActuationMode::DirectJointPosition;
    }
    return MujocoJointActuationMode::PdTorque;
}

double clip_hinge_q(mjModel const* m, int jid, double q) {
    if (m == nullptr || jid < 0) {
        return q;
    }
    if (m->jnt_limited[jid] == 0) {
        return q;
    }
    double const lo = m->jnt_range[2 * jid];
    double const hi = m->jnt_range[2 * jid + 1];
    return mju_clip(q, lo, hi);
}

}  // namespace

FeixiMujocoHardware::~FeixiMujocoHardware() {
    viewer_running_.store(false);
    if (viewer_thread_.joinable()) {
        viewer_thread_.join();
    }
    free_mujoco();
}

double FeixiMujocoHardware::parse_double_or(
    //从参数map中获取参数值，没有的话使用默认值
    std::unordered_map<std::string, std::string> const& pmap, char const* key,
    double def_v) {
    auto it = pmap.find(key);
    if (it == pmap.end()) return def_v;
    auto v = trimmed_stod(it->second.c_str());
    return v ? *v : def_v;
}

bool FeixiMujocoHardware::parse_bool_or(std::unordered_map<std::string, std::string> const& pmap,
                                        char const* key, bool def_v) {
    auto it = pmap.find(key);
    if (it == pmap.end()) {
        return def_v;
    }
    std::string s;
    s.reserve(it->second.size());
    for (unsigned char const uc : it->second) {
        if (!std::isspace(uc)) {
            s.push_back(static_cast<char>(std::tolower(uc)));
        }
    }
    if (s.empty()) {
        return def_v;
    }
    return s == "1" || s == "true" || s == "yes" || s == "on";
}

int FeixiMujocoHardware::parse_int_or(std::unordered_map<std::string, std::string> const& pmap,
                                      char const* key, int def_v) {
    auto it = pmap.find(key);
    if (it == pmap.end()) {
        return def_v;
    }
    try {
        return std::stoi(it->second);
    } catch (...) {
        return def_v;
    }
}

void FeixiMujocoHardware::free_mujoco() {
    if (data_) {
        mj_deleteData(data_);
        data_ = nullptr;
    }
    if (model_) {
        mj_deleteModel(model_);
        model_ = nullptr;
    }
    model_loaded_ = false;
}

hardware_interface::CallbackReturn FeixiMujocoHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    //获取硬件信息，通过get_hardware_info()获取硬件信息，然后通过info.hardware_parameters获取参数map
    hardware_interface::HardwareInfo const& info = get_hardware_info();
    auto const& pmap = info.hardware_parameters;
    auto it = pmap.find("mjcf_model_path");
    if (it == pmap.end() || it->second.empty()) {
        RCLCPP_ERROR(get_logger(), "Missing hardware param 'mjcf_model_path'.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    std::string const mjcf_abs = [&] {
        std::filesystem::path p(it->second);
        if (!p.empty() && p.is_relative()) {
            return std::filesystem::absolute(p).string();
        }
        return it->second;
    }();
    //解析参数，从参数map中获取参数值，没有的话使用默认值   
    arm_kp_ = parse_double_or(pmap, "arm_kp", 1200.0);
    arm_kd_ = parse_double_or(pmap, "arm_kd", 90.0);
    gripper_kp_ = parse_double_or(pmap, "gripper_ctrl_kp", 380.0);

    effort_command_mode_ = false;
    auto cmd_it = pmap.find("command_interface");
    if (cmd_it != pmap.end()) {
        std::string v;
        v.reserve(cmd_it->second.size());
        for (unsigned char uc : cmd_it->second) {
            if (std::isspace(uc) == 0) {
                v.push_back(static_cast<char>(std::tolower(uc)));
            }
        }
        effort_command_mode_ = (v == "effort");
    }

    joint_actuation_mode_ = parse_joint_actuation_mode(pmap);
    if (effort_command_mode_ && joint_actuation_mode_ == MujocoJointActuationMode::DirectJointPosition) {
        RCLCPP_WARN(get_logger(),
                    "mujoco_joint_actuation=direct needs position command_interface; using pd_torque.");
        joint_actuation_mode_ = MujocoJointActuationMode::PdTorque;
    }

    enable_viewer_ = parse_bool_or(pmap, "enable_mujoco_viewer", false);
    viewer_width_ = parse_int_or(pmap, "viewer_width", 900);
    viewer_height_ = parse_int_or(pmap, "viewer_height", 700);
    if (viewer_width_ < 320) {
        viewer_width_ = 320;
    }
    if (viewer_height_ < 240) {
        viewer_height_ = 240;
    }

    free_mujoco();
    char err[1024] = {0};
    model_ = mj_loadXML(mjcf_abs.c_str(), nullptr, err, sizeof(err));
    if (!model_) {
        RCLCPP_ERROR(get_logger(), "mj_loadXML failed: %s\nPath=%s", err, mjcf_abs.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    data_ = mj_makeData(model_);
    for (int a = 0; a < model_->nu; ++a) {
        data_->ctrl[a] = 0.0;
    }
    model_loaded_ = true;

    for (int i = 0; i < 7; ++i) {
        std::string jn = "joint" + std::to_string(i + 1);
        mj_arm_joint_ids_[i] = mj_name2id(model_, mjOBJ_JOINT, jn.c_str());
        if (mj_arm_joint_ids_[i] < 0) {
            RCLCPP_ERROR(get_logger(), "MuJoCo joint '%s' not found.", jn.c_str());
            free_mujoco();
            return hardware_interface::CallbackReturn::ERROR;
        }
        std::string an = "m_joint" + std::to_string(i + 1);
        mj_arm_act_ids_[i] = mj_name2id(model_, mjOBJ_ACTUATOR, an.c_str());
        if (mj_arm_act_ids_[i] < 0) {
            RCLCPP_ERROR(get_logger(), "MuJoCo actuator '%s' not found.", an.c_str());
            free_mujoco();
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    mj_gripper_joint_ = mj_name2id(model_, mjOBJ_JOINT, "left_driver_joint");
    if (mj_gripper_joint_ < 0) {
        RCLCPP_ERROR(get_logger(), "MuJoCo gripper hinge 'left_driver_joint' not found.");
        free_mujoco();
        return hardware_interface::CallbackReturn::ERROR;
    }
    mj_gripper_act_ = mj_name2id(model_, mjOBJ_ACTUATOR, "fingers_actuator");
    if (mj_gripper_act_ < 0) {
        RCLCPP_ERROR(get_logger(), "MuJoCo actuator 'fingers_actuator' not found.");
        free_mujoco();
        return hardware_interface::CallbackReturn::ERROR;
    }

    init_pose_q_ = default_init_pose_q();
    auto pose_it = pmap.find("init_pose_q");
    if (pose_it != pmap.end() && !pose_it->second.empty()) {
        std::array<double, 8> parsed{};
        if (parse_csv_doubles8(pose_it->second, &parsed)) {
            init_pose_q_ = parsed;
        } else {
            RCLCPP_WARN(get_logger(),
                        "Could not parse init_pose_q (expected 8 comma-separated floats). Using default.");
        }
    }

    int init_lock_writes = parse_int_or(pmap, "init_lock_writes", 500);
    if (init_lock_writes < 0) {
        init_lock_writes = 0;
    }
    init_lock_writes_remaining_ = init_lock_writes;

    apply_arm_gripper_pose_to_mujoco(init_pose_q_);
    RCLCPP_INFO(get_logger(), "MuJoCo loaded: %s", mjcf_abs.c_str());
    if (joint_actuation_mode_ == MujocoJointActuationMode::DirectJointPosition) {
        RCLCPP_INFO(get_logger(), "mujoco_joint_actuation=direct (kinematic joint targets after mj_step).");
    }
    if (init_lock_writes_remaining_ > 0) {
        RCLCPP_INFO(get_logger(), "init: arm+gripper seeded; init_lock_writes=%d (kinematic hold).",
                    init_lock_writes_remaining_);
    }

    if (enable_viewer_) {
        viewer_running_.store(true);
        viewer_thread_ = std::thread([this]() { viewer_main(); });
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FeixiMujocoHardware::read(const rclcpp::Time&,
                                                           const rclcpp::Duration&) {
    std::lock_guard<std::mutex> lock(mj_mutex_);
    if (!model_loaded_) {
        return hardware_interface::return_type::ERROR;
    }

    for (int i = 0; i < 7; ++i) {
        int const jid = mj_arm_joint_ids_[i];
        int const qa = model_->jnt_qposadr[jid];
        int const va = model_->jnt_dofadr[jid];
        std::string const jn = "joint" + std::to_string(i + 1);
        set_state(jn + "/" + hardware_interface::HW_IF_POSITION, data_->qpos[qa]);
        set_state(jn + "/" + hardware_interface::HW_IF_VELOCITY, data_->qvel[va]);
    }

    int const gql = model_->jnt_qposadr[mj_gripper_joint_];
    int const gdl = model_->jnt_dofadr[mj_gripper_joint_];
    set_state(std::string("gripper_finger_joint") + "/" + hardware_interface::HW_IF_POSITION,
              data_->qpos[gql]);
    set_state(std::string("gripper_finger_joint") + "/" + hardware_interface::HW_IF_VELOCITY,
              data_->qvel[gdl]);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FeixiMujocoHardware::write(const rclcpp::Time&,
                                                            const rclcpp::Duration& period) {
    std::lock_guard<std::mutex> lock(mj_mutex_);
    if (!model_loaded_) {
        return hardware_interface::return_type::ERROR;
    }

    if (init_lock_writes_remaining_ > 0) {
        snap_arm_gripper_to_init_pose();
    }

    std::array<double, 8> q_cmd = init_pose_q_;
    for (int i = 0; i < 7; ++i) {
        std::string const jkey = "joint" + std::to_string(i + 1);
        std::string const pos_iface = jkey + "/" + hardware_interface::HW_IF_POSITION;
        if (!effort_command_mode_ && init_lock_writes_remaining_ <= 0 && has_command(pos_iface)) {
            double const cmd = get_command<double>(pos_iface);
            if (std::isfinite(cmd)) {
                q_cmd[static_cast<std::size_t>(i)] = cmd;
            }
        }
        q_cmd[static_cast<std::size_t>(i)] =
            clip_hinge_q(model_, mj_arm_joint_ids_[i], q_cmd[static_cast<std::size_t>(i)]);
    }
    {
        std::string const g_pos =
            std::string("gripper_finger_joint") + "/" + hardware_interface::HW_IF_POSITION;
        if (!effort_command_mode_ && init_lock_writes_remaining_ <= 0 && has_command(g_pos)) {
            double const c = get_command<double>(g_pos);
            if (std::isfinite(c)) {
                q_cmd[7] = c;
            }
        }
        q_cmd[7] = clip_hinge_q(model_, mj_gripper_joint_, q_cmd[7]);
    }

    bool const use_direct = (!effort_command_mode_ &&
                             joint_actuation_mode_ == MujocoJointActuationMode::DirectJointPosition);

    if (use_direct) {
        for (int i = 0; i < 7; ++i) {
            int const aid = mj_arm_act_ids_[i];
            data_->ctrl[aid] = 0.0;
        }
        {
            int const aid = mj_gripper_act_;
            double const lo = model_->actuator_ctrlrange[2 * aid];
            double const hi = model_->actuator_ctrlrange[2 * aid + 1];
            data_->ctrl[aid] = 0.5 * (lo + hi);
        }
    } else {
        for (int i = 0; i < 7; ++i) {
            int const jid = mj_arm_joint_ids_[i];
            int const aid = mj_arm_act_ids_[i];
            int const qa = model_->jnt_qposadr[jid];
            int const va = model_->jnt_dofadr[jid];
            double q = data_->qpos[qa];
            double qdot = data_->qvel[va];

            std::string const jkey = "joint" + std::to_string(i + 1);
            std::string const pos_iface = jkey + "/" + hardware_interface::HW_IF_POSITION;
            std::string const eff_iface = jkey + "/" + hardware_interface::HW_IF_EFFORT;

            double lo = model_->actuator_ctrlrange[2 * aid];
            double hi = model_->actuator_ctrlrange[2 * aid + 1];
            double tau = 0.0;

            double const q_des = q_cmd[static_cast<std::size_t>(i)];

            if (has_command(eff_iface)) {
                double const c = get_command<double>(eff_iface);
                tau = std::isfinite(c) ? c : 0.0;
            } else if (!effort_command_mode_) {
                tau = arm_kp_ * (q_des - q) - arm_kd_ * qdot;
                if (!std::isfinite(tau)) {
                    tau = 0.0;
                }
            }

            data_->ctrl[aid] = mju_clip(tau, lo, hi);
        }

        {
            int const gql = model_->jnt_qposadr[mj_gripper_joint_];
            double gq = data_->qpos[gql];
            std::string const g_pos =
                std::string("gripper_finger_joint") + "/" + hardware_interface::HW_IF_POSITION;
            std::string const g_eff =
                std::string("gripper_finger_joint") + "/" + hardware_interface::HW_IF_EFFORT;

            int const aid = mj_gripper_act_;
            double lo = model_->actuator_ctrlrange[2 * aid];
            double hi = model_->actuator_ctrlrange[2 * aid + 1];

            if (has_command(g_eff)) {
                double const c = get_command<double>(g_eff);
                double const tau_g = std::isfinite(c) ? c : 0.0;
                data_->ctrl[aid] = mju_clip(tau_g, lo, hi);
            } else if (!effort_command_mode_) {
                double const g_cmd = q_cmd[7];
                double err = g_cmd - gq;
                double ctrl_ff = 0.5 * (lo + hi);
                double u = ctrl_ff + gripper_kp_ * err;
                if (!std::isfinite(u)) {
                    u = ctrl_ff;
                }
                data_->ctrl[aid] = mju_clip(u, lo, hi);
            } else {
                double ctrl_ff = 0.5 * (lo + hi);
                data_->ctrl[aid] = mju_clip(ctrl_ff, lo, hi);
            }
        }
    }

    double const Ts = model_->opt.timestep;
    double dur = period.seconds();
    if (!std::isfinite(dur) || dur <= 0.0) {
        dur = Ts > 0.0 ? Ts : 0.002;
    }
    // startup / overruns can pass a huge period once; avoid stepping months of sim in one write.
    double const dur_cap = 0.02;
    if (dur > dur_cap) {
        dur = dur_cap;
    }
    int n_steps = Ts > 0.0 ? static_cast<int>(std::ceil(dur / Ts)) : 1;
    if (n_steps < 1) n_steps = 1;
    if (n_steps > 16384) n_steps = 16384;

    for (int s = 0; s < n_steps; ++s) {
        mj_step(model_, data_);
    }

    if (use_direct) {
        apply_arm_gripper_pose_to_mujoco(q_cmd);
    }

    if (init_lock_writes_remaining_ > 0) {
        --init_lock_writes_remaining_;
    }

    return hardware_interface::return_type::OK;
}

void FeixiMujocoHardware::apply_arm_gripper_pose_to_mujoco(std::array<double, 8> const& q) {
    for (int i = 0; i < 7; ++i) {
        int const jid = mj_arm_joint_ids_[i];
        int const qa = model_->jnt_qposadr[jid];
        int const va = model_->jnt_dofadr[jid];
        data_->qpos[qa] = q[static_cast<std::size_t>(i)];
        data_->qvel[va] = 0.0;
    }
    int const gql = model_->jnt_qposadr[mj_gripper_joint_];
    int const gdl = model_->jnt_dofadr[mj_gripper_joint_];
    data_->qpos[gql] = q[7];
    data_->qvel[gdl] = 0.0;
    mj_forward(model_, data_);
}

void FeixiMujocoHardware::snap_arm_gripper_to_init_pose() {
    apply_arm_gripper_pose_to_mujoco(init_pose_q_);
}

void FeixiMujocoHardware::viewer_main() {
    MjView view;
    if (!view.init(model_, viewer_width_, viewer_height_, "Feixi MuJoCo (ROS 2)")) {
        RCLCPP_ERROR(get_logger(),
                     "MuJoCo viewer init failed (need DISPLAY / GPU and libglfw). Running headless.");
        return;
    }
    view.install_interaction(model_);
    view.init_camera(MjCameraPreset::Default);

    while (viewer_running_.load(std::memory_order_relaxed) && view.window != nullptr &&
           !glfwWindowShouldClose(view.window)) {
        glfwMakeContextCurrent(view.window);
        {
            std::lock_guard<std::mutex> lock(mj_mutex_);
            if (!model_loaded_ || model_ == nullptr || data_ == nullptr) {
                break;
            }
            int w = 0;
            int h = 0;
            glfwGetFramebufferSize(view.window, &w, &h);
            mjrRect const viewport = {0, 0, w, h};
            mjv_updateScene(model_, data_, &view.opt, nullptr, &view.cam, mjCAT_ALL, &view.scn);
            mjr_render(viewport, &view.scn, &view.con);
        }
        glfwSwapBuffers(view.window);
        glfwPollEvents();
    }
}

}  // namespace feixi_ros2_control

PLUGINLIB_EXPORT_CLASS(feixi_ros2_control::FeixiMujocoHardware, hardware_interface::SystemInterface)
