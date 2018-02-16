#include "RobotModel.hpp"
#include <Configuration.h>
RobotModel* RobotModel::getRobotModel() {
    static RobotModel robot;
    return & robot;
}

RobotModel::RobotModel() {
    dart::utils::DartLoader urdfLoader;
    mSkel = urdfLoader.parseSkeleton(THIS_COM"RobotModel/valkyrie.urdf");
    mInitialPositions = mSkel->getPositions();

    printf("[Robot Model] Constructed\n");
}

RobotModel::~RobotModel() {}

void RobotModel::updateModel(const Eigen::VectorXd & q_,
                             const Eigen::VectorXd & qdot_) {
    mSkel->setPositions(q_);
    mSkel->setVelocities(qdot_);
}

Eigen::VectorXd RobotModel::getInitialPositions() {
    return mInitialPositions;
}
dart::dynamics::SkeletonPtr RobotModel::getSkeleton() {
    return mSkel;
}

int RobotModel::getNumDofs() {
    return mSkel->getNumDofs();
}

Eigen::MatrixXd RobotModel::getMassMatrix() {
    return mSkel->getMassMatrix();
}

Eigen::VectorXd RobotModel::getCoriolisAndGravityForces() {
    return mSkel->getCoriolisAndGravityForces();
}

double RobotModel::getTotalMass() {
    return mSkel->getMass();
}

Eigen::VectorXd RobotModel::getCOM(const dart::dynamics::Frame* _wrt) {
    return mSkel->getCOM(_wrt);
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> RobotModel::getJointPositionLimits() {
    return std::make_pair(mSkel->getPositionLowerLimits(),
           mSkel->getPositionUpperLimits());
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> RobotModel::getJointVelocityLimits() {
    return std::make_pair(mSkel->getPositionLowerLimits(),
           mSkel->getPositionUpperLimits());
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> RobotModel::getJointAccelerationLimits() {
    return std::make_pair(mSkel->getPositionLowerLimits(),
           mSkel->getPositionUpperLimits());
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> RobotModel::getJointForceLimits() {
    return std::make_pair(mSkel->getForceLowerLimits(),
           mSkel->getForceUpperLimits());
}
