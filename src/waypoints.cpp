// ros
#include <ros/ros.h>
// ros package to access package directory
#include <ros/package.h>

/*
 * main function
 */
int main( int argc, char **argv )
{
  /**
   * Initialize ROS
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init( argc, argv, "navi_goals_node" );

  // spin ROS
  try
  {
    // run a loop continuously to allow the callbacks to be called when a new message arrives
    ros::spin();
  }
  catch( std::runtime_error& e )
  {
    ROS_ERROR( "ros spin failed: %s", e.what() );
    return -1;
  }

  return 0;
}