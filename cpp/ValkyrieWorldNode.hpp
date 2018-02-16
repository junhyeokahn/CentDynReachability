#ifndef VALKYRIE_WORLD_NODE
#define VALKYRIE_WORLD_NODE

#include <dart/dart.hpp>
#include <dart/io/io.hpp>
#include <dart/gui/osg/osg.hpp>
#include <Eigen/Dense>

class RobotModel;

class ValkyrieWorldNode : public dart::gui::osg::WorldNode
{
private:
    dart::simulation::WorldPtr mWorld;
    RobotModel* mRobot;
    dart::dynamics::SkeletonPtr mSkel;
    std::vector< Eigen::VectorXd > mJPosLimits;
    std::vector< Eigen::VectorXd > mJVelLimits;
    std::vector< Eigen::VectorXd > mJAccLimits;
    std::vector< Eigen::VectorXd > mJTrqLimits;

    void _getJointLimits(RobotModel* robot_, std::vector< Eigen::VectorXd > &jPosLimits_,
                         std::vector< Eigen::VectorXd > &jVelLimits_,
                         std::vector< Eigen::VectorXd > &jAccLimits_,
                         std::vector< Eigen::VectorXd > &jTrqLimits_);

    Eigen::VectorXd _drawSample(const std::vector< Eigen::VectorXd > & limits_,
                                double var_=-1.0);
public:
    ValkyrieWorldNode(const dart::simulation::WorldPtr & world);
    virtual ~ValkyrieWorldNode();

    void customPreStep() override;
};

#endif /* VALKYRIE_WORLD_NODE */
