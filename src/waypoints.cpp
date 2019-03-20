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
// tf2 matrix
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

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
  ros::init( argc, argv, "navi_goals_node" );  // initialize roscpp node
  
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n; // start the roscpp node by creating a ros handle
  
  /// declares an array of waypoint
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
    
  // declare message type of 'point' which is used to define each waypoint's coordinates (double)
  geometry_msgs::PointStamped point;

  // Waypoint Nr. 0
  point.point.x = 5.0;
  point.point.y = 5.0;
  point.point.z = -45.0;
  waypoints.push_back(point);
  
  // Waypoint Nr. 1
  point.point.x = 2.0;
  point.point.y = -3.0;
  point.point.z = -135.0;
  waypoints.push_back(point);
  
  // Waypoint Nr. 2
  point.point.x = 8.0;
  point.point.y = -2.0;
  point.point.z = 90.0;
  waypoints.push_back(point);
  
  // Waypoint Nr. 3
  point.point.x = 8.0;
  point.point.y = 5.0;
  point.point.z = 90.0;
  waypoints.push_back(point);
  
  // Waypoint Nr. 4
  point.point.x = -6.0;
  point.point.y = 6.0;
  point.point.z = -135.0;
  waypoints.push_back(point);
  
  // Waypoint Nr. 5
  point.point.x = -6.0;
  point.point.y = -1.5;
  point.point.z = 0.0;
  waypoints.push_back(point);
  
  // Waypoint Nr. 6
  point.point.x = 0.0;
  point.point.y = 0.0;
  point.point.z = 45.0;
  waypoints.push_back(point);

  for( int j=0; j<waypoints.size(); j++ )
  {
    // declare message type for goal
    move_base_msgs::MoveBaseGoal goal; // evaluate use of the type MoveBaseActionGoal rather than MoveBaseGoal
    // use "robot_footprint" to send a goal relative to the robot actual pose and "map" to send an absolute goal
    goal.target_pose.header.frame_id = "map"; // or use "base_link"
    goal.target_pose.header.stamp= ros::Time::now();
    goal.target_pose.pose.position.x= waypoints[j].point.x;
    goal.target_pose.pose.position.y= waypoints[j].point.y;
    goal.target_pose.pose.position.z= 0.0;

    // convert the degrees to quaternion
    double yaw= waypoints[j].point.z*M_PI/180.;
    tf2::Quaternion q;
    q.setRPY( 0., 0., yaw );
    goal.target_pose.pose.orientation.x= q.x();
    goal.target_pose.pose.orientation.y= q.y();
    goal.target_pose.pose.orientation.z= q.z();
    goal.target_pose.pose.orientation.w= q.w();

    ROS_INFO("Sending goal: (%.2f, %.2f, %.2f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.z );
    action_client.sendGoal( goal );
    action_client.waitForResult(ros::Duration(60.0)); // 60 sec is the allocated timeout to continue with the next goal
    // action_client.waitForResult(); // if the result message doesn't make it to the action client, then waitForResult() is going to block forever
    
    if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base move to (%.2f, %.2f, %.2f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.z );
    else
      ROS_INFO("The base failed to move to (%.2f, %.2f, %.2f) for some reason", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.z );
    ros::Duration(0.5).sleep();
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