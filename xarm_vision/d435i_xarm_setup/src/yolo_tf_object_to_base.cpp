/* Copyright 2025 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * YOLO 3D -> TF bridge
 *
 * Subscribes to /yolo/detections_3d (yolo_msgs/DetectionArray)
 * and publishes a TF from the YOLO 3D bounding box center to a
 * fixed child frame (e.g. "leaf").
 *
 * Author: adapted from ORK version
 */

#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "yolo_msgs/msg/detection_array.hpp"

rclcpp::Logger logger = rclcpp::get_logger("yolo_tf_obj_to_base");
std::shared_ptr<rclcpp::Node> node;
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

/**
 * Callback for YOLO 3D detections.
 * Topic: /yolo/detections_3d
 * Type : yolo_msgs/msg/DetectionArray
 */
void detections_callback(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
{
  // No detections
  if (msg->detections.empty())
    return;

  // More then one detection- for now the logic is just to refer to the first one
  if (msg->detections.size() > 1) {
    RCLCPP_WARN(
      logger,
      "Received %zu detections, only broadcasting the first one",
      msg->detections.size());
  }

  // Take the first detection
  const auto &det = msg->detections.at(0);

  // Make sure there is a 3D bounding box
  // (Detect3DNode fills det.bbox3d when use_3d:=True)
  const auto &bbox3d = det.bbox3d;

  // If size is zero, it's probably not a valid 3D bbox
  if (bbox3d.size.x == 0.0 &&
      bbox3d.size.y == 0.0 &&
      bbox3d.size.z == 0.0)
  {
    RCLCPP_WARN(logger, "Detection has no valid 3D bounding box, skipping");
    return;
  }

  // Position of the object = center of the 3D bounding box
  tf2::Vector3 vector(
    bbox3d.center.position.x,
    bbox3d.center.position.y,
    bbox3d.center.position.z);

  // Orientation: Detect3DNode does not explicitly set it, so it will
  // typically be identity (0,0,0,1). We still pass whatever is in the msg.
  tf2::Quaternion quaternion(
    bbox3d.center.orientation.x,
    bbox3d.center.orientation.y,
    bbox3d.center.orientation.z,
    bbox3d.center.orientation.w);

  // If the quaternion is zero-length (just in case), force identity
  if (quaternion.length2() == 0.0) {
    quaternion.setValue(0.0, 0.0, 0.0, 1.0);
  }

  tf2::Transform transform(quaternion, vector);

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = node->get_clock()->now();

  // Parent frame:
  // Detect3DNode sets bbox3d.frame_id = target_frame (e.g. "link_base"),
  // so we try to use that. If empty, fall back to "link_base".
  if (!bbox3d.frame_id.empty()) {
    transform_stamped.header.frame_id = bbox3d.frame_id;
  } else {
    transform_stamped.header.frame_id = "link_base";   // fallback
  }

  // Child frame: the detected object (rename to whatever you like)
  transform_stamped.child_frame_id = "leaf";

  tf2::toMsg(transform, transform_stamped.transform);

  tf_broadcaster->sendTransform(transform_stamped);

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  // Allows parameters like use_sim_time to be set from the launch file
  node_options.automatically_declare_parameters_from_overrides(true);

  node = rclcpp::Node::make_shared("yolo_tf_object_to_base", node_options);

  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

  // Subscribe to YOLO 3D detections
  // If you change the namespace in yolo.launch.py, keep this topic in sync
  auto sub = node->create_subscription<yolo_msgs::msg::DetectionArray>(
    "/yolo/detections_3d",          // topic
    10,                             // queue size
    &detections_callback);          // callback

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

