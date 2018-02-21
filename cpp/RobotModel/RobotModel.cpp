#include "RobotModel.hpp"
#include <Configuration.h>
RobotModel* RobotModel::getRobotModel() {
    static RobotModel robot;
    return & robot;
}

RobotModel::RobotModel() {
    dart::utils::DartLoader urdfLoader;
    mSkel = urdfLoader.parseSkeleton(THIS_COM"RobotModel/valkyrie_simple.urdf");
    mInitialPositions = mSkel->getPositions();
    mNumDof = mSkel->getNumDofs();
    mIcent = Eigen::MatrixXd::Zero(6, 6);
    mJcent = Eigen::MatrixXd::Zero(6, mNumDof);
    mAcent = Eigen::MatrixXd::Zero(6, mNumDof);

    printf("[Robot Model] Constructed\n");
}

RobotModel::~RobotModel() {}

void RobotModel::updateModel(const Eigen::VectorXd & q_,
                             const Eigen::VectorXd & qdot_) {
    mSkel->setPositions(q_);
    mSkel->setVelocities(qdot_);
    _updateCentroidFrame(q_, qdot_);
}

void RobotModel::_updateCentroidFrame(const Eigen::VectorXd & q_,
                                      const Eigen::VectorXd & qdot_) {

    Eigen::MatrixXd Jsp = Eigen::MatrixXd::Zero(6, mNumDof);
    Eigen::VectorXd p_gl = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd R_gl = Eigen::MatrixXd::Zero(3, 3);
    Eigen::VectorXd pCoM_g = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd I = Eigen::MatrixXd::Zero(6, 6);
    Eigen::Isometry3d T_lc = Eigen::Isometry3d::Identity();
    Eigen::MatrixXd AdT_lc = Eigen::MatrixXd::Zero(6, 6);
    mIcent = Eigen::MatrixXd::Zero(6, 6);
    mJcent = Eigen::MatrixXd::Zero(6, mNumDof);
    mAcent = Eigen::MatrixXd::Zero(6, mNumDof);

    for (int i = 0; i < mSkel->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNodePtr bn = mSkel->getBodyNode(i);
        Jsp = mSkel->getJacobian(bn);
        p_gl = bn->getWorldTransform().translation();
        R_gl = bn->getWorldTransform().linear();
        pCoM_g = mSkel->getCOM();
        I = bn->getSpatialInertia();
        T_lc.linear() = R_gl.transpose();
        T_lc.translation() = R_gl.transpose() * (pCoM_g - p_gl);
        AdT_lc = dart::math::getAdTMatrix(T_lc);
        mIcent += AdT_lc.transpose() * I * AdT_lc;
        mAcent += AdT_lc.transpose() * I * Jsp;
    }
    mJcent = mIcent.inverse() * mAcent;
}

Eigen::MatrixXd RobotModel::getCentroidInertia() {
    return mIcent;
}

Eigen::MatrixXd RobotModel::getCentroidJacobian() {
    return mJcent;
}

dart::dynamics::SkeletonPtr RobotModel::getSkeleton() {
    return mSkel;
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

int RobotModel::getNumDofs() {
    return mSkel->getNumDofs();
}

Eigen::VectorXd RobotModel::getInitialPositions() {
    return mInitialPositions;
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
