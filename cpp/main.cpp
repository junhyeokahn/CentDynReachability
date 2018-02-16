#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include "ValkyrieWorldNode.hpp"
#include "Configuration.h"

int main(int argc, char *argv[])
{
    //// Generate world and add skeletons
    dart::simulation::WorldPtr world(new dart::simulation::World);
    dart::utils::DartLoader urdfLoader;
    dart::dynamics::SkeletonPtr robot = urdfLoader.parseSkeleton(
            THIS_COM"RobotModel/valkyrie.urdf");
    world->addSkeleton(robot);
    robot->setSelfCollisionCheck(true);
    robot->setAdjacentBodyCheck(false);
    Eigen::Vector3d gravity(0.0, 0.0, 0.0);
    world->setGravity(gravity);
    world->setTimeStep(1.0/1000);

    //// Wrap a worldnode
    osg::ref_ptr<ValkyrieWorldNode> node
        = new ValkyrieWorldNode(world);
    node->setNumStepsPerCycle(20);

    //// Create viewer
    dart::gui::osg::Viewer viewer;
    viewer.addWorldNode(node);
    viewer.simulate(false);
    std::cout << "=====================================" << std::endl;
    std::cout << viewer.getInstructions() << std::endl;
    viewer.setUpViewInWindow(0, 0, 640, 480);
    viewer.getCameraManipulator()->setHomePosition(
            ::osg::Vec3( 2.57,  3.14, 2.)*2.0,
            ::osg::Vec3( 0.00,  0.00, 1.00),
            ::osg::Vec3(-0.24, -0.25, 0.94));
    viewer.setCameraManipulator(viewer.getCameraManipulator());
    viewer.run();

    return 0;
}
