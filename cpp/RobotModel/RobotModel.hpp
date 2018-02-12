#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include <dart/dart.hpp>
#include <dart/io/io.hpp>
#include <dart/io/urdf/urdf.hpp>
#include <Eigen/Dense>

class RobotModel
{
private:
    RobotModel();

    dart::dynamics::SkeletonPtr mSkel;

public:
    static RobotModel* getRobotModel();
    virtual ~RobotModel();

    void updateModel(const Eigen::VectorXd & q_, const Eigen::VectorXd & qdot_);
    int getNumDofs();
    dart::dynamics::SkeletonPtr getSkeleton(); // will be depricated
    Eigen::MatrixXd getMassMatrix();
    double getTotalMass();
    Eigen::VectorXd getCoriolisAndGravityForces();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> getJointPositionLimits();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> getJointVelocityLimits();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> getJointAccelerationLimits();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> getJointForceLimits();


};

#endif /* ROBOTMODEL_H */
