#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/buffer.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle n;

  ros::AsyncSpinner spinner(1);
  spinner.start();

    static const std::string PLANNING_GROUP_ARM = "arm";
    
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);

    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
    move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

      //1. random pose goal
      // move_group_interface_arm.setRandomTarget();
      // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

      // geometry_msgs::PoseStamped current_pose;
      // current_pose = move_group_interface_arm.getCurrentPose("tool0_1");

      // bool success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

      // move_group_interface_arm.move();

      // //get current pose values
      // geometry_msgs::Pose target_pose1;
      // target_pose1.orientation = current_pose.pose.orientation;
      // std::cout<<current_pose.pose.position<<std::endl;

      // // get the current joint values
      // std::vector<double> joint_values;
      // move_group_interface_arm.getCurrentState()->copyJointGroupPositions(move_group_interface_arm.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_interface_arm.getName()), joint_values);

      // ROS_INFO_STREAM("Current joint values:");
      // ROS_INFO_STREAM("Joint 1 " << ": " << joint_values[0] << " m ");
      // for (size_t i = 1; i < joint_values.size(); ++i)
      // {
      //   ROS_INFO_STREAM("Joint " << i+1 << ": " << joint_values[i]*59.296<<" degrees ");
      // }

    //2. for generating pose and joint values
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;

      geometry_msgs::PoseStamped current_pose;
      current_pose = move_group_interface_arm.getCurrentPose("tool0_1");

      geometry_msgs::Pose target_pose1;
      target_pose1.orientation = current_pose.pose.orientation;
      target_pose1.position = current_pose.pose.position;

      target_pose1.position.x = 0;//-0.17595,-0.404246,0.473335
      target_pose1.position.y = -0.45;
      target_pose1.position.z = 0.225618;
      move_group_interface_arm.setPoseTarget(target_pose1);

      bool success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

      move_group_interface_arm.move();

      //get current pose values
      target_pose1.orientation = current_pose.pose.orientation;
      std::cout<<current_pose.pose.position<<std::endl;

      // get the current joint values
      std::vector<double> joint_values;
      move_group_interface_arm.getCurrentState()->copyJointGroupPositions(move_group_interface_arm.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_interface_arm.getName()), joint_values);

      ROS_INFO_STREAM("Current joint values:");
      ROS_INFO_STREAM("Joint 1 " << ": " << joint_values[0] << " m ");
      for (size_t i = 1; i < joint_values.size(); ++i)
      {
        ROS_INFO_STREAM("Joint " << i+1 << ": " << joint_values[i]*59.296<<" degrees ");
      }

    //3. for displaying only pose values
    // geometry_msgs::PoseStamped current_pose;
    // current_pose = move_group_interface_arm.getCurrentPose("tool0_1");

    // geometry_msgs::Pose target_pose1;
    // target_pose1.orientation = current_pose.pose.orientation;
    // target_pose1.position = current_pose.pose.position;
    // std::cout<<target_pose1.position.x<<","<<target_pose1.position.y<<","<<target_pose1.position.z<<std::endl;
    // std::cout<<target_pose1.orientation.w<<","<<target_pose1.orientation.x<<","<<target_pose1.orientation.y<<","<<target_pose1.orientation.z<<std::endl;


  ros::shutdown();
  return 0;
}
