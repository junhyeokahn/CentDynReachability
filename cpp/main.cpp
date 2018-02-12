#include "RobotModel.hpp"
#include <Eigen/Dense>

void _getJointLimits(RobotModel* robot_, std::vector< Eigen::VectorXd > &jPosLimits_,
                                         std::vector< Eigen::VectorXd > &jVelLimits_,
                                         std::vector< Eigen::VectorXd > &jAccLimits_,
                                         std::vector< Eigen::VectorXd > &jTrqLimits_) {
    jPosLimits_[0] = robot_->getJointPositionLimits().first;
    jPosLimits_[1] = robot_->getJointPositionLimits().second;
    jVelLimits_[0] = robot_->getJointVelocityLimits().first;
    jVelLimits_[1] = robot_->getJointVelocityLimits().second;
    jAccLimits_[0] = robot_->getJointAccelerationLimits().first;
    jAccLimits_[1] = robot_->getJointAccelerationLimits().second;
    jTrqLimits_[0] = robot_->getJointForceLimits().first;
    jTrqLimits_[1] = robot_->getJointForceLimits().second;
}

Eigen::VectorXd _drawSample(const std::vector< Eigen::VectorXd > & limits_, 
                            double var_=-1.0) {
    Eigen::VectorXd ret = Eigen::VectorXd::Zero(limits_[0].size());
    // draw
    if (var_ == -1.0) {
        
    } else {
        
    }
    return ret;
}

int main(int argc, char *argv[])
{
    RobotModel* robot = RobotModel::getRobotModel();
    std::vector< Eigen::VectorXd > jPosLimits(2);
    std::vector< Eigen::VectorXd > jVelLimits(2);
    std::vector< Eigen::VectorXd > jAccLimits(2);
    std::vector< Eigen::VectorXd > jTrqLimits(2);
    _getJointLimits(robot, jPosLimits, jVelLimits, jAccLimits, jTrqLimits);
    Eigen::VectorXd jPos = _drawSample(jPosLimits);

    std::cout << "done" << std::endl;
    return 0;
}
