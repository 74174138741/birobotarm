#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#if __has_include(<mujoco/mujoco.h>)
#include <mujoco/mujoco.h>
#else
#include <mujoco.h>
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "hardware_interface/system_interface.hpp"
#pragma GCC diagnostic pop

namespace feixi_ros2_control {

/** How joint POSITION commands are realized in MuJoCo (ros2_control command_interface=position). */
enum class MujocoJointActuationMode {
    /** PD computes motor torques (physical). */
    PdTorque,
    /** After mj_step, overwrite arm + driver DOF qpos to match commands (kinematic / ideal position). */
    DirectJointPosition,
};

class FeixiMujocoHardware final : public hardware_interface::SystemInterface {
   public:
    ~FeixiMujocoHardware() override;

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;

    hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;

    hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

   private:
    void free_mujoco();
    void viewer_main();
    void apply_arm_gripper_pose_to_mujoco(std::array<double, 8> const& q);
    void snap_arm_gripper_to_init_pose();
    static double parse_double_or(std::unordered_map<std::string, std::string> const& pmap,
                                  char const* key, double def_v);
    static bool parse_bool_or(std::unordered_map<std::string, std::string> const& pmap,
                              char const* key, bool def_v);
    static int parse_int_or(std::unordered_map<std::string, std::string> const& pmap,
                            char const* key, int def_v);

    mjModel* model_{nullptr};
    mjData* data_{nullptr};

    std::array<int, 7> mj_arm_joint_ids_{};
    std::array<int, 7> mj_arm_act_ids_{};
    int mj_gripper_joint_{-1};
    int mj_gripper_act_{-1};

    double arm_kp_{1200.0};
    double arm_kd_{90.0};
    double gripper_kp_{2000.0};

    std::array<double, 8> init_pose_q_{};
    int init_lock_writes_remaining_{0};
    bool effort_command_mode_{false};
    MujocoJointActuationMode joint_actuation_mode_{MujocoJointActuationMode::PdTorque};

    bool model_loaded_{false};

    std::mutex mj_mutex_;
    std::atomic<bool> viewer_running_{false};
    std::thread viewer_thread_;
    bool enable_viewer_{false};
    int viewer_width_{900};
    int viewer_height_{700};
};

}  // namespace feixi_ros2_control
