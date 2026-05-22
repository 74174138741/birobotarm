#include "feixi_ros2_control/feixi_pinocchio_dynamics.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace feixi_ros2_control {

bool FeixiPinocchioDynamics::loadUrdf(std::string const& urdf_path, std::string const& ee_frame,
                                      std::string& err) {
    model_loaded_ = false;
    try {
        pinocchio::Model m;
        pinocchio::urdf::buildModel(urdf_path, m);
        model_ = std::move(m);
        data_ = pinocchio::Data(model_);
        ee_frame_id_ = model_.getFrameId(ee_frame);
        model_loaded_ = true;
        err.clear();
        return true;
    } catch (std::exception const& ex) {
        err = ex.what();
        return false;
    }
}

void FeixiPinocchioDynamics::inverseDynamics(Eigen::VectorXd const& q, Eigen::VectorXd const& v,
                                             Eigen::VectorXd const& a, Eigen::VectorXd& tau) {
    tau = pinocchio::rnea(model_, data_, q, v, a);
}

void FeixiPinocchioDynamics::jacobianTransposeWrenchLocalWorldAligned(
    Eigen::VectorXd const& q, Eigen::Matrix<double, 6, 1> const& f_spatial, Eigen::VectorXd& tau) {
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model_.nv);
    J.setZero();
    pinocchio::getFrameJacobian(model_, data_, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);
    tau.noalias() = J.transpose() * f_spatial;
}

}  // namespace feixi_ros2_control
