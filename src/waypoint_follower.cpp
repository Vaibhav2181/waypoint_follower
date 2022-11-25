#include <stdlib.h>
#include <deque>
#include <ros/ros.h>
#include <sstream>
#include <move_base_msgs/MoveBaseAction.h>         //move_base action
#include <actionlib/client/simple_action_client.h> //actionlib
#include <tf/transform_broadcaster.h>

// for series waypoint
std::deque<geometry_msgs::PoseStamped> waypointSegments;
geometry_msgs::PoseStamped waypoints;
float dist;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
move_base_msgs::MoveBaseGoal goal;

void done_cbf(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result_ptr) // action done callback
{
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("The robot has completed the task!");
    }
    else
    {
        ROS_INFO("The robot has failed to approch the goal.");
        if (state == actionlib::SimpleClientGoalState::ABORTED)
            ROS_INFO("The mission has been aborted.");
    }
}

void active_cbf() // action active callback
{
    ROS_INFO("The robot starts working.");
}

void feedback_cbf(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback_ptr) // action feedback callback
{
    dist = sqrt(pow(feedback_ptr->base_position.pose.position.x - goal.target_pose.pose.position.x, 2) + pow(feedback_ptr->base_position.pose.position.y - goal.target_pose.pose.position.y, 2));
    ROS_INFO("Robot is at :(%.2f,%.2f) co-ordinate ", feedback_ptr->base_position.pose.position.x, feedback_ptr->base_position.pose.position.y);
    ROS_INFO("Distance : .%.2f remaining to the goal", dist);
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &poseIn)
{
    waypointSegments.push_back(*poseIn);
    ROS_INFO("Wait for waypoint goal..");

    if (waypointSegments.size())
    {
        waypoints = waypointSegments.front();
        std::cout << waypointSegments.size() << std::endl;

        MoveBaseClient ac("move_base", true);  // action name
        ac.waitForServer(ros::Duration(10.0)); // wait for 5 seconds
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = waypoints.pose.position.x;
        goal.target_pose.pose.position.y = waypoints.pose.position.y;
        goal.target_pose.pose.position.z = waypoints.pose.position.z;

        tf::Quaternion tfq;
        geometry_msgs::Quaternion msgq;
        tfq.setRPY(0.0, 0.0, 0.0); // rpy to quaternion

        double yaw = tf::getYaw(waypoints.pose.orientation);
        goal.target_pose.pose.orientation.w = yaw;
        ROS_INFO("Sending goal...");

        ac.sendGoal(goal, done_cbf, active_cbf, feedback_cbf); // send goal

        bool finished_within_time = ac.waitForResult(ros::Duration(20.0));
        if (!finished_within_time)
            ROS_INFO("robot taking too much time.. taking next goal..");
    }
    waypointSegments.pop_back();
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "move_base_simple_client");
    ros::NodeHandle nh;

    ros::Subscriber WaypointSub = nh.subscribe("/waypoints", 50, pose_callback);

    ros::spin();
    return 0;
}
