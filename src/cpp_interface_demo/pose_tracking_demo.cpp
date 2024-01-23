/*******************************************************************************
 *      Title     : pose_tracking_example.cpp
 *      Project   : moveit_servo
 *      Created   : 09/04/2020
 *      Author    : Adam Pettinger
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.pose_tracking_demo");

// Class for monitoring status of moveit_servo
class StatusMonitor
{
public:
  StatusMonitor(const rclcpp::Node::SharedPtr& node, const std::string& topic)
  {
    sub_ = node->create_subscription<std_msgs::msg::Int8>(topic, rclcpp::SystemDefaultsQoS(),
                                                          [this](const std_msgs::msg::Int8::ConstSharedPtr& msg) {
                                                            return statusCB(msg);
                                                          });
  }

private:
  void statusCB(const std_msgs::msg::Int8::ConstSharedPtr& msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      RCLCPP_INFO_STREAM(LOGGER, "Servo status: " << status_str);
    }
  }

  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_;
};

/**
 * Instantiate the pose tracking interface.
 * Send a pose slightly different from the starting pose
 * Then keep updating the target pose a little bit
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("pose_tracking_demo");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node);

  if (servo_parameters == nullptr)
  {
    RCLCPP_FATAL(LOGGER, "Could not get servo parameters!");
    exit(EXIT_FAILURE);
  }

  // Load the planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor->providePlanningSceneService();
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor(servo_parameters->joint_topic);
  planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  // Wait for Planning Scene Monitor to setup
  if (!planning_scene_monitor->waitForCurrentRobotState(node->now(), 5.0 /* seconds */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Error waiting for current robot state in PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  // Create the pose tracker
  moveit_servo::PoseTracking tracker(node, servo_parameters, planning_scene_monitor);

  //
  bool is_initial_imu_received = false;
  geometry_msgs::msg::PoseStamped imu_teleop_pose, initial_imu_teleop_pose;
  auto target_pose_sub =
      node->create_subscription<geometry_msgs::msg::PoseStamped>("imu_teleop_pose", rclcpp::SystemDefaultsQoS(),
                                                                 [&](const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
                                                                 { imu_teleop_pose = *msg; 
                                                                 if(!is_initial_imu_received) {initial_imu_teleop_pose = imu_teleop_pose; is_initial_imu_received = true;}});

  // Make a publisher for sending pose commands
  auto target_pose_pub =
      node->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", rclcpp::SystemDefaultsQoS());

  // Subscribe to servo status (and log it when it changes)
  StatusMonitor status_monitor(node, servo_parameters->status_topic);

  Eigen::Vector3d lin_tol{ 0.000001, 0.000001, 0.000001 };
  double rot_tol = 0.00001;

  // Run the pose tracking in a new thread
  std::thread move_to_pose_thread([&tracker, &lin_tol, &rot_tol] {
    moveit_servo::PoseTrackingStatusCode tracking_status =
        tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */);
    RCLCPP_INFO_STREAM(LOGGER, "Pose tracker exited with status: "
                                   << moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(tracking_status));
  });

    // Get the current EE transform
  geometry_msgs::msg::TransformStamped current_ee_tf;
  tracker.getCommandFrameTransform(current_ee_tf);

  // Convert it to a Pose
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = current_ee_tf.header.frame_id;
  target_pose.pose.position.x = current_ee_tf.transform.translation.x;
  target_pose.pose.position.y = current_ee_tf.transform.translation.y;
  target_pose.pose.position.z = current_ee_tf.transform.translation.z;
  target_pose.pose.orientation = current_ee_tf.transform.rotation;

  rclcpp::WallRate loop_rate(100);
  for (size_t i = 0; i < 100; ++i)
  {
    target_pose.pose.position.z += 0.0004;
    target_pose.header.stamp = node->now();
    target_pose_pub->publish(target_pose);

    loop_rate.sleep();
  }
  tracker.resetTargetPose();

  tf2::Quaternion current_ee_quat, imu_teleop_quat, initial_imu_teleop_quat;
  tf2::convert(current_ee_tf.transform.rotation, current_ee_quat);
  tf2::Quaternion rotx_180 = tf2::Quaternion(tf2::Vector3(1, 0, 0), M_PI);
  

  // 처음 base 기준ee orientation과 invesed_robot_heading의 orientation의 차이를 initial_ee_offset
  // 
  
  while (rclcpp::ok()) {
    if(is_initial_imu_received){
      
      target_pose.pose.position.x = current_ee_tf.transform.translation.x + imu_teleop_pose.pose.position.x - initial_imu_teleop_pose.pose.position.x;
      target_pose.pose.position.y = current_ee_tf.transform.translation.y + imu_teleop_pose.pose.position.y - initial_imu_teleop_pose.pose.position.y;
      target_pose.pose.position.z = current_ee_tf.transform.translation.z + imu_teleop_pose.pose.position.z - initial_imu_teleop_pose.pose.position.z;

      tf2::convert(initial_imu_teleop_pose.pose.orientation, initial_imu_teleop_quat);
      initial_imu_teleop_quat *= rotx_180;
      tf2::convert(imu_teleop_pose.pose.orientation, imu_teleop_quat);
      imu_teleop_quat *= rotx_180;
      
      auto target_quat = current_ee_quat * initial_imu_teleop_quat.inverse() * imu_teleop_quat;
      tf2::convert(target_quat, target_pose.pose.orientation);
      target_pose.header.stamp = node->now();
    }
    target_pose_pub->publish(target_pose);
    loop_rate.sleep();
  }
  // Make sure the tracker is stopped and clean up
  move_to_pose_thread.join();

  // Kill executor thread before shutdown
  executor.cancel();
  executor_thread.join();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
