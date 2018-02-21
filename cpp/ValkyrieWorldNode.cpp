#include "ValkyrieWorldNode.hpp"
#include "RobotModel.hpp"
#include "Utilities.hpp"

ValkyrieWorldNode::ValkyrieWorldNode(const dart::simulation::WorldPtr & world_) :
    dart::gui::osg::WorldNode(world_) {

    mNumData = 20000;
    mCount = 0;
    mWorld = world_;
    mSkel = world_->getSkeleton("valkyrie");
    mRobot = RobotModel::getRobotModel();
    mJPosLimits.resize(2);
    mJVelLimits.resize(2);
    mJAccLimits.resize(2);
    mJTrqLimits.resize(2);
    _getJointLimits(mRobot, mJPosLimits, mJVelLimits, mJAccLimits, mJTrqLimits);
}

ValkyrieWorldNode::~ValkyrieWorldNode() {}

void ValkyrieWorldNode::customPreStep() {
    // Draw Sample and Update Robot //
    Eigen::VectorXd jPos = _drawSample(mJPosLimits);
    Eigen::VectorXd jVel = _drawSample(mJVelLimits, 1.0);
    Eigen::VectorXd jAcc = _drawSample(mJAccLimits, 1.0);
    mRobot->updateModel(jPos, jVel);

    // Self Collision Check //
    // If there is no collision, save CoM data //
    // Otherwise, clear the collision and reset to initial position //
    auto collisionEngine = mWorld->getConstraintSolver()->getCollisionDetector();
    auto collisionGroup = mWorld->getConstraintSolver()->getCollisionGroup();

    dart::collision::CollisionOption option;
    dart::collision::CollisionResult result;
    bool collision = collisionGroup->collide(option, &result);
    if (collision) {
        std::cout << "Collision is Occured" << std::endl;
        std::unordered_set<const dart::dynamics::BodyNode*> collidingBN
            = result.getCollidingBodyNodes();
        for (std::unordered_set<const dart::dynamics::BodyNode*>::const_iterator iter = collidingBN.begin();
                iter != collidingBN.end(); ++iter ) {
            std::cout << (*iter)->getName() << std::endl;
        }
        result.clear();
        jPos.head(6) = Eigen::VectorXd::Zero(6);
        jPos[5] = 1.5;
        mSkel->setPositions(mRobot->getInitialPositions());
        result.clear();
    } else {
        std::cout << "No Collision" << std::endl;
        // Construct Reference Contact Frame //
        Eigen::Isometry3d iso( Eigen::Isometry3d::Identity() );
        auto bn = mRobot->getSkeleton()->getBodyNode("rightFoot");
        iso = bn->getWorldTransform();
        dart::dynamics::SimpleFrame refFrame
            (dart::dynamics::Frame::World(), "rFootFrame", iso);

        // Get CoM w.r.t Reference Frame //
        std::cout << mRobot->getCOM(&refFrame) << std::endl;
        myUtils::saveVector(mRobot->getCOM(&refFrame), "CoM_rFoot");

        // For Visualization Purpose //
        jPos.head(6) = Eigen::VectorXd::Zero(6);
        jPos[5] = 1.5;
        mSkel->setPositions(jPos);
        ++mCount;
    }

    if (mCount > mNumData) {
        std::cout << mNumData << " is Saved" << std::endl;
        exit(0);
    }
    //std::cout << "----" << std::endl;
}

void ValkyrieWorldNode::_getJointLimits(RobotModel* robot_, std::vector< Eigen::VectorXd > &jPosLimits_,
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

Eigen::VectorXd ValkyrieWorldNode::_drawSample(const std::vector< Eigen::VectorXd > & limits_,
                            double var_) {
    int numVars(limits_[0].size());
    Eigen::VectorXd ret = Eigen::VectorXd::Zero(numVars);

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

    if (var_ == -1.0) {
        for (int i = 0; i < numVars; ++i) {
            if (isinf(limits_[0][i]) || isinf(limits_[1][i]) ) {
                std::uniform_real_distribution<double> dis(0., 2*M_PI);
                ret[i] = dis(gen);
            } else {
                std::uniform_real_distribution<double> dis(limits_[0][i], limits_[1][i]);
                ret[i] = dis(gen);
            }
        }
    } else {
        for (int i = 0; i < numVars; ++i) {
            if (isinf(limits_[0][i]) || isinf(limits_[1][i]) ) {
                std::normal_distribution<double> dis(0.0, var_);
                ret[i] = dis(gen);
            } else {
                std::normal_distribution<double> dis(limits_[0][i] + limits_[1][i],
                                                     var_);
                ret[i] = dis(gen);
            }
        }
    }
    return ret;
}

