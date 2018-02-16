#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <Eigen/Dense>

class RobotModel
{
private:
    RobotModel();

    dart::dynamics::SkeletonPtr mSkel;
    Eigen::VectorXd mInitialPositions;

public:
    static RobotModel* getRobotModel();
    virtual ~RobotModel();

    void updateModel(const Eigen::VectorXd & q_, const Eigen::VectorXd & qdot_);
    int getNumDofs();
    dart::dynamics::SkeletonPtr getSkeleton(); // will be depricated
    Eigen::MatrixXd getMassMatrix();
    double getTotalMass();
    Eigen::VectorXd getCOM(const dart::dynamics::Frame* _wrt=dart::dynamics::Frame::World());
    Eigen::VectorXd getCoriolisAndGravityForces();
    Eigen::VectorXd getInitialPositions();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> getJointPositionLimits();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> getJointVelocityLimits();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> getJointAccelerationLimits();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> getJointForceLimits();


};

#endif /* ROBOTMODEL_H */
