#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

    static const std::string PLANNING_GROUP_ARM = "arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
    move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

    //adding the shelf as a collision object
    //moveit_msgs::CollisionObject collision_object;
    //collision_object.header.frame_id = move_group_interface_arm.getPlanningFrame();
    //collision_object.id = "box1";

    // Define a box to add to the world.
    //shape_msgs::SolidPrimitive primitive;
    //primitive.type = primitive.BOX;
    // primitive.dimensions.resize(3);
    // primitive.dimensions[primitive.BOX_X] = 1.0;
    // primitive.dimensions[primitive.BOX_Y] = 1.0;
    // primitive.dimensions[primitive.BOX_Z] = 1.0;

    // // Define a pose for the box (specified relative to frame_id)
    // geometry_msgs::Pose box_pose;
    // box_pose.orientation.w = 1.0;
    // box_pose.position.x = 1.0;
    // box_pose.position.y = 1.0;
    // box_pose.position.z = 1.0;

    // collision_object.primitives.push_back(primitive);
    // collision_object.primitive_poses.push_back(box_pose);
    // collision_object.operation = collision_object.ADD;

    // std::vector<moveit_msgs::CollisionObject> collision_objects;
    // collision_objects.push_back(collision_object);

    // // Now, let's add the collision object into the world
    // // (using a vector that could contain additional objects)
    // ROS_INFO_NAMED("tutorial", "Add an object into the world");
    // planning_scene_interface.addCollisionObjects(collision_objects);

    // 1. Move to pick position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("pick"));
    bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    ros::Duration(2.0).sleep();

    moveit_msgs::CollisionObject object_to_attach;
    object_to_attach.header.frame_id = move_group_interface_arm.getPlanningFrame();
    object_to_attach.id = "box2";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.20;
    primitive.dimensions[primitive.BOX_Y] = 0.10;
    primitive.dimensions[primitive.BOX_Z] = 0.20;

    // We define the frame/pose for this box so that it appears in the gripper
    object_to_attach.header.frame_id = move_group_interface_arm.getEndEffectorLink();
    geometry_msgs::Pose grab_pose;
    grab_pose.orientation.w = 1.0;
    grab_pose.position.z = -0.15;

    // First, we add the object to the world (without using a vector)
    object_to_attach.primitives.push_back(primitive);
    object_to_attach.primitive_poses.push_back(grab_pose);
    object_to_attach.operation = object_to_attach.ADD;
    planning_scene_interface.applyCollisionObject(object_to_attach);

    // Then, we attach the object to the robot at the given link and allow collisions between the object and the links
    ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
    move_group_interface_arm.attachObject(object_to_attach.id, "tool0_1", { "tool0_1" });

    // //move the tool tip away to avoid collision
    // geometry_msgs::PoseStamped current_pose;
    // current_pose = move_group_interface_arm.getCurrentPose("tool0_1");
    
    // geometry_msgs::Pose target_pose1;
    // target_pose1.orientation = current_pose.pose.orientation;
    // target_pose1.position = current_pose.pose.position;
    // float y1 = target_pose1.position.x;
    // target_pose1.position.y = y1 - 0.1;
    // move_group_interface_arm.setPoseTarget(target_pose1);
    // move_group_interface_arm.move();

    // 2. Place the TCP (Tool Center Point, the tip of the robot) above the blue box
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("place"));
    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    ros::Duration(2.0).sleep();

    ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
    move_group_interface_arm.detachObject(object_to_attach.id);

    ROS_INFO_NAMED("tutorial", "Remove the objects from the world");
    std::vector<std::string> object_ids;
    object_ids.push_back(object_to_attach.id);
    planning_scene_interface.removeCollisionObjects(object_ids);

    //3. Go to home position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();
    ros::Duration(2.0).sleep();

  ros::shutdown();
  return 0;
}
