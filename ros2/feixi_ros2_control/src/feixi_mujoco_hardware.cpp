#include "feixi_ros2_control/feixi_mujoco_hardware.hpp"

#include "feixi_ros2_control/feixi_pinocchio_dynamics.hpp"
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

#include <Eigen/Core>

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

PinocchioDynamicsMode parse_pinocchio_mode(std::unordered_map<std::string, std::string> const& pmap) {
    auto const it = pmap.find("dynamics_mode");
    if (it == pmap.end() || it->second.empty()) {
        return PinocchioDynamicsMode::None;
    }
    std::string v;
    v.reserve(it->second.size());
    for (unsigned char uc : it->second) {
        if (std::isspace(uc) == 0) {
            v.push_back(static_cast<char>(std::tolower(uc)));
        }
    }
    if (v == "none" || v == "off" || v == "0") {
        return PinocchioDynamicsMode::None;
    }
    if (v == "acceleration" || v == "accel") {
        return PinocchioDynamicsMode::Acceleration;
    }
    if (v == "position" || v == "position_only") {
        return PinocchioDynamicsMode::PositionOnly;
    }
    if (v == "trajectory" || v == "full" || v == "pos_vel_accel") {
        return PinocchioDynamicsMode::Trajectory;
    }
    if (v == "cartesian_wrench" || v == "cartesian" || v == "wrench") {
        return PinocchioDynamicsMode::CartesianWrench;
    }
    return PinocchioDynamicsMode::None;
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
    wrench_sub_.reset();
    wrench_node_.reset();
    pinocchio_dyn_.reset();
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

    pinocchio_mode_ = parse_pinocchio_mode(pmap);
    auto pit = pmap.find("pinocchio_urdf_path");
    if (pit != pmap.end()) {
        pinocchio_urdf_path_ = pit->second;
    }
    if (!pinocchio_urdf_path_.empty() && std::filesystem::path(pinocchio_urdf_path_).is_relative()) {
        pinocchio_urdf_path_ = std::filesystem::absolute(pinocchio_urdf_path_).string();
    }

    auto ee_it = pmap.find("pinocchio_ee_frame");
    if (ee_it != pmap.end() && !ee_it->second.empty()) {
        pinocchio_ee_frame_ = ee_it->second;
    }

    if (pinocchio_mode_ != PinocchioDynamicsMode::None) {
        if (effort_command_mode_) {
            RCLCPP_ERROR(get_logger(),
                         "dynamics_mode is set but command_interface=effort; Pinocchio needs position-style "
                         "command interfaces.");
            free_mujoco();
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint_actuation_mode_ == MujocoJointActuationMode::DirectJointPosition) {
            RCLCPP_ERROR(get_logger(),
                         "dynamics_mode is incompatible with mujoco_joint_actuation=direct.");
            free_mujoco();
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (pinocchio_urdf_path_.empty()) {
            RCLCPP_ERROR(get_logger(), "dynamics_mode requires hardware param pinocchio_urdf_path.");
            free_mujoco();
            return hardware_interface::CallbackReturn::ERROR;
        }
        pinocchio_dyn_ = std::make_unique<FeixiPinocchioDynamics>();
        std::string perr;
        if (!pinocchio_dyn_->loadUrdf(pinocchio_urdf_path_, pinocchio_ee_frame_, perr)) {
            RCLCPP_ERROR(get_logger(), "Pinocchio URDF load failed: %s", perr.c_str());
            free_mujoco();
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (pinocchio_dyn_->nv() != 8) {
            RCLCPP_ERROR(get_logger(), "Pinocchio model nv=%d (expected 8 for this stack).",
                         pinocchio_dyn_->nv());
            pinocchio_dyn_.reset();
            free_mujoco();
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (pinocchio_mode_ == PinocchioDynamicsMode::CartesianWrench) {
            std::string topic = "/feixi/cartesian_wrench";
            auto tit = pmap.find("cartesian_wrench_topic");
            if (tit != pmap.end() && !tit->second.empty()) {
                topic = tit->second;
            }
            wrench_node_ = std::make_shared<rclcpp::Node>("feixi_mujoco_wrench_rx");
            wrench_sub_ = wrench_node_->create_subscription<geometry_msgs::msg::Wrench>(
                topic, rclcpp::QoS(1).best_effort(),
                [this](geometry_msgs::msg::Wrench::UniquePtr msg) {
                    std::lock_guard<std::mutex> lk(wrench_mutex_);
                    wrench_cmd_[0] = msg->torque.x;
                    wrench_cmd_[1] = msg->torque.y;
                    wrench_cmd_[2] = msg->torque.z;
                    wrench_cmd_[3] = msg->force.x;
                    wrench_cmd_[4] = msg->force.y;
                    wrench_cmd_[5] = msg->force.z;
                });
            RCLCPP_INFO(get_logger(), "Pinocchio Cartesian wrench topic (geometry_msgs/Wrench): %s",
                        topic.c_str());
        }

        RCLCPP_INFO(get_logger(), "Pinocchio inverse dynamics enabled (URDF=%s, ee_frame=%s).",
                    pinocchio_urdf_path_.c_str(), pinocchio_ee_frame_.c_str());
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
        bool const use_pinocchio =
            (!effort_command_mode_ && pinocchio_mode_ != PinocchioDynamicsMode::None &&
             pinocchio_dyn_ != nullptr && pinocchio_dyn_->ok());

        std::optional<Eigen::VectorXd> tau_pin;

        if (use_pinocchio) {
            if (wrench_node_) {
                rclcpp::spin_some(wrench_node_);
            }

            Eigen::VectorXd q;
            Eigen::VectorXd v;
            mujoco_joint_state_to_pinocchio(q, v);
            Eigen::VectorXd q_des;
            Eigen::VectorXd v_des;
            Eigen::VectorXd a_des;
            read_joint_commands_into_desired(q_cmd, q_des, v_des, a_des);

            Eigen::VectorXd tau(8);
            if (pinocchio_mode_ == PinocchioDynamicsMode::CartesianWrench) {
                Eigen::Matrix<double, 6, 1> f;
                {
                    std::lock_guard<std::mutex> lk(wrench_mutex_);
                    for (int k = 0; k < 6; ++k) {
                        f(static_cast<Eigen::Index>(k)) = wrench_cmd_[static_cast<std::size_t>(k)];
                    }
                }
                pinocchio_dyn_->jacobianTransposeWrenchLocalWorldAligned(q, f, tau);
            } else {
                Eigen::VectorXd const e = q_des - q;
                Eigen::VectorXd const ed = v_des - v;
                Eigen::VectorXd a_cmd(8);
                switch (pinocchio_mode_) {
                    case PinocchioDynamicsMode::Acceleration:
                        a_cmd = a_des + arm_kp_ * e + arm_kd_ * ed;
                        break;
                    case PinocchioDynamicsMode::PositionOnly:
                        a_cmd = arm_kp_ * e + arm_kd_ * ed;
                        break;
                    case PinocchioDynamicsMode::Trajectory:
                        a_cmd = a_des + arm_kp_ * e + arm_kd_ * ed;
                        break;
                    default:
                        a_cmd.setZero();
                        break;
                }
                a_cmd[7] = gripper_kp_ * (q_des[7] - q[7]) - arm_kd_ * v[7];
                pinocchio_dyn_->inverseDynamics(q, v, a_cmd, tau);
            }

            tau_pin = std::move(tau);

            for (int i = 0; i < 7; ++i) {
                int const aid = mj_arm_act_ids_[i];
                double lo = model_->actuator_ctrlrange[2 * aid];
                double hi = model_->actuator_ctrlrange[2 * aid + 1];
                double t = (*tau_pin)[i];
                if (!std::isfinite(t)) {
                    t = 0.0;
                }
                data_->ctrl[aid] = mju_clip(t, lo, hi);
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
        }

        {
            int const gql = model_->jnt_qposadr[mj_gripper_joint_];
            double gq = data_->qpos[gql];
            std::string const g_eff =
                std::string("gripper_finger_joint") + "/" + hardware_interface::HW_IF_EFFORT;

            int const aid = mj_gripper_act_;
            double lo = model_->actuator_ctrlrange[2 * aid];
            double hi = model_->actuator_ctrlrange[2 * aid + 1];

            if (tau_pin.has_value()) {
                double t = (*tau_pin)[7];
                if (!std::isfinite(t)) {
                    t = 0.0;
                }
                data_->ctrl[aid] = mju_clip(t, lo, hi);
            } else if (has_command(g_eff)) {
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

void FeixiMujocoHardware::mujoco_joint_state_to_pinocchio(Eigen::VectorXd& q, Eigen::VectorXd& v) {
    q.resize(8);
    v.resize(8);
    for (int i = 0; i < 7; ++i) {
        int const jid = mj_arm_joint_ids_[i];
        int const qa = model_->jnt_qposadr[jid];
        int const va = model_->jnt_dofadr[jid];
        q[i] = data_->qpos[qa];
        v[i] = data_->qvel[va];
    }
    int const gql = model_->jnt_qposadr[mj_gripper_joint_];
    int const gdl = model_->jnt_dofadr[mj_gripper_joint_];
    q[7] = data_->qpos[gql];
    v[7] = data_->qvel[gdl];
}

void FeixiMujocoHardware::read_joint_commands_into_desired(std::array<double, 8> const& q_cmd,
                                                           Eigen::VectorXd& q_des, Eigen::VectorXd& v_des,
                                                           Eigen::VectorXd& a_des) {
    Eigen::VectorXd q_now(8);
    Eigen::VectorXd v_now(8);
    mujoco_joint_state_to_pinocchio(q_now, v_now);
    q_des = q_now;
    v_des = Eigen::VectorXd::Zero(8);
    a_des = Eigen::VectorXd::Zero(8);

    for (int i = 0; i < 7; ++i) {
        std::string const jkey = "joint" + std::to_string(i + 1);
        std::string const pos_iface = jkey + "/" + hardware_interface::HW_IF_POSITION;
        if (has_command(pos_iface)) {
            double const c = get_command<double>(pos_iface);
            if (std::isfinite(c)) {
                q_des[i] = c;
            }
        }
        std::string const vel_iface = jkey + "/" + hardware_interface::HW_IF_VELOCITY;
        if (has_command(vel_iface)) {
            double const c = get_command<double>(vel_iface);
            if (std::isfinite(c)) {
                v_des[i] = c;
            }
        }
        std::string const acc_iface = jkey + "/" + hardware_interface::HW_IF_ACCELERATION;
        if (has_command(acc_iface)) {
            double const c = get_command<double>(acc_iface);
            if (std::isfinite(c)) {
                a_des[i] = c;
            }
        }
    }
    std::string const g_pos =
        std::string("gripper_finger_joint") + "/" + hardware_interface::HW_IF_POSITION;
    if (has_command(g_pos)) {
        double const c = get_command<double>(g_pos);
        if (std::isfinite(c)) {
            q_des[7] = c;
        }
    } else {
        q_des[7] = q_cmd[7];
    }
    std::string const g_vel =
        std::string("gripper_finger_joint") + "/" + hardware_interface::HW_IF_VELOCITY;
    if (has_command(g_vel)) {
        double const c = get_command<double>(g_vel);
        if (std::isfinite(c)) {
            v_des[7] = c;
        }
    }
    std::string const g_acc =
        std::string("gripper_finger_joint") + "/" + hardware_interface::HW_IF_ACCELERATION;
    if (has_command(g_acc)) {
        double const c = get_command<double>(g_acc);
        if (std::isfinite(c)) {
            a_des[7] = c;
        }
    }
}

}  // namespace feixi_ros2_control

PLUGINLIB_EXPORT_CLASS(feixi_ros2_control::FeixiMujocoHardware, hardware_interface::SystemInterface)
