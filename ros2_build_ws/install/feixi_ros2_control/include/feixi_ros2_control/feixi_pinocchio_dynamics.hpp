#pragma once

#include <string>

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace feixi_ros2_control {

class FeixiPinocchioDynamics {
   public:
    FeixiPinocchioDynamics() = default;

    /** Load Pinocchio model from a standalone URDF (already expanded, not .xacro). */
    bool loadUrdf(std::string const& urdf_path, std::string const& ee_frame, std::string& err);

    int nv() const { return model_.nv; }

    /** Inverse dynamics: tau = RNEA(q, v, a) = M(q) a + b(q, v). */
    void inverseDynamics(Eigen::VectorXd const& q, Eigen::VectorXd const& v, Eigen::VectorXd const& a,
                         Eigen::VectorXd& tau);

    /**
     * tau = J^T * f_spatial with f_spatial = [n_x,n_y,n_z, f_x,f_y,f_z]^T (Pinocchio spatial force),
     * consistent with getFrameJacobian(..., LOCAL_WORLD_ALIGNED).
     */
    void jacobianTransposeWrenchLocalWorldAligned(Eigen::VectorXd const& q,
                                                  Eigen::Matrix<double, 6, 1> const& f_spatial,
                                                  Eigen::VectorXd& tau);

    bool ok() const { return model_loaded_; }

   private:
    pinocchio::Model model_{};
    pinocchio::Data data_;
    bool model_loaded_{false};
    pinocchio::FrameIndex ee_frame_id_{0};
};

}  // namespace feixi_ros2_control
