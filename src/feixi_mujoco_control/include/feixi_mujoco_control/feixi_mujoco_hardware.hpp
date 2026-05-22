#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <Eigen/Core>

#include "geometry_msgs/msg/wrench.hpp"
#include "rclcpp/rclcpp.hpp"

#if __has_include(<mujoco/mujoco.h>)
#include <mujoco/mujoco.h>
#else
#include <mujoco.h>
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "hardware_interface/system_interface.hpp"
#pragma GCC diagnostic pop

namespace feixi_mujoco_control {

class FeixiPinocchioDynamics;

/** MuJoCo actuation: maps to ros2_control command_interface (position / velocity / effort). */
enum class MujocoJointActuationMode {
    Position,
    Velocity,
    Torque,
    /** Kinematic qpos snap after mj_step (position commands only). */
    Direct,
};

/** Computed joint torques via Pinocchio inverse dynamics (optional). */
enum class PinocchioDynamicsMode : std::uint8_t {
    None = 0,
    /** tau = M (ddq_des + Kp e + Kd e_dot) + C + g */
    Acceleration,
    /** tau = M (Kp e + Kd e_dot) + C + g */
    PositionOnly,
    /** tau = M (ddq_des + Kp e + Kd e_dot) + C + g with full trajectory command */
    Trajectory,
    /** tau = J^T f (subscribes to geometry_msgs/Wrench) */
    CartesianWrench,
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
    void mujoco_joint_state_to_pinocchio(Eigen::VectorXd& q, Eigen::VectorXd& v);
    void read_joint_commands_into_desired(std::array<double, 8> const& q_cmd, Eigen::VectorXd& q_des,
                                          Eigen::VectorXd& v_des, Eigen::VectorXd& a_des);
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
    double arm_kv_{90.0};
    double gripper_kp_{2000.0};

    std::array<double, 8> init_pose_q_{};
    int init_lock_writes_remaining_{0};
    MujocoJointActuationMode joint_actuation_mode_{MujocoJointActuationMode::Position};

    PinocchioDynamicsMode pinocchio_mode_{PinocchioDynamicsMode::None};
    std::unique_ptr<FeixiPinocchioDynamics> pinocchio_dyn_;
    std::string pinocchio_urdf_path_;
    std::string pinocchio_ee_frame_{"link7"};
    std::shared_ptr<rclcpp::Node> wrench_node_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_sub_;
    std::mutex wrench_mutex_;
    std::array<double, 6> wrench_cmd_{};

    bool model_loaded_{false};

    std::mutex mj_mutex_;
    std::atomic<bool> viewer_running_{false};
    std::thread viewer_thread_;
    bool enable_viewer_{false};
    int viewer_width_{900};
    int viewer_height_{700};
};

}  // namespace feixi_mujoco_control
