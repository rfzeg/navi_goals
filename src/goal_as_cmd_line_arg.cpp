# Node that reads-in command line arguments for a x,y goal position and sends it to move_base via topic message

include "ros/ros.h"
include "geometry_msgs/PoseStamped.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_publisher");
    ros::NodeHandle n;
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Rate loop_rate(1);
    geometry_msgs::PoseStamped msg;
    if (argc != 3) {
        ROS_INFO("Please provide x and y of the goal!");
    }
    msg.header.frame_id = "/map";
    msg.pose.position.x = atol(argv[1]);
    msg.pose.position.y = atol(argv[2]);
    msg.pose.position.z = 0.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;
    ROS_INFO("Sending Robot to %f, %f", msg.pose.position.x, msg.pose.position.y);
    goal_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    return 0;
}
