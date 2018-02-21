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

    /*
     * Update Ig, Ag, Jg
     * , where \dot{h} = I_{cm} * \dot{x}_{cm}
     *                 = A_{cm} * \dot{q}
     *          J_{cm} = I_{cm}^{-1} * A_{cm}
     *    \dot{x}_{cm} = J_{cm} * \dot{q}
     */
    void _updateCentroidFrame(const Eigen::VectorXd & q_,
                              const Eigen::VectorXd & qdot_);

    dart::dynamics::SkeletonPtr mSkel;
    int mNumDof;
    Eigen::VectorXd mInitialPositions;
    Eigen::MatrixXd mIcent;
    Eigen::MatrixXd mJcent;
    Eigen::MatrixXd mAcent;

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
    Eigen::MatrixXd getCentroidInertia();
    Eigen::MatrixXd getCentroidJacobian();
    Eigen::VectorXd getInitialPositions();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> getJointPositionLimits();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> getJointVelocityLimits();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> getJointAccelerationLimits();
    std::pair<Eigen::VectorXd, Eigen::VectorXd> getJointForceLimits();


};

#endif /* ROBOTMODEL_H */
