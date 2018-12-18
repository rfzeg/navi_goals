// ros
#include <ros/ros.h>
// ros package to access package directory
#include <ros/package.h>
// move base
#include <move_base_msgs/MoveBaseAction.h>
// simple move action
#include <actionlib/client/simple_action_client.h>
// stamped point message
#include <geometry_msgs/PointStamped.h>

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
  
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  
  /// array of waypoint
  std::vector<geometry_msgs::PointStamped> waypoints;
                  
  // create a convenience typedef for a SimpleActionClient that will allow us to communicate with actions that adhere to the MoveBaseAction action interface
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
                  
  // tell the action client that we want to spin a thread by default
  MoveBaseClient action_client( "move_base", true );
  
  // wait for the action server to come up
  while( !action_client.waitForServer( ros::Duration(5.0) ) )
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    

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