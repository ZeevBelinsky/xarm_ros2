/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include "xarm_planner/xarm_planner.h"
#include <std_msgs/msg/bool.hpp>
#include <xarm_msgs/srv/plan_pose.hpp>
#include <xarm_msgs/srv/plan_pose_weighted.hpp>
#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <xarm_msgs/srv/plan_single_straight.hpp>
#include <std_srvs/srv/trigger.hpp>


#define BIND_CLS_CB(func) std::bind(func, this, std::placeholders::_1, std::placeholders::_2)

class XArmPlannerRunner
{
public:
	XArmPlannerRunner(rclcpp::Node::SharedPtr& node);
	~XArmPlannerRunner() {};

private:
	void setup_servers();

	bool do_pose_plan(const std::shared_ptr<xarm_msgs::srv::PlanPose::Request> req, std::shared_ptr<xarm_msgs::srv::PlanPose::Response> res);
	bool do_pose_plan_weighted(const std::shared_ptr<xarm_msgs::srv::PlanPoseWeighted::Request> req, std::shared_ptr<xarm_msgs::srv::PlanPoseWeighted::Response> res);
	bool do_joint_plan(const std::shared_ptr<xarm_msgs::srv::PlanJoint::Request> req, std::shared_ptr<xarm_msgs::srv::PlanJoint::Response> res);
	bool do_single_cartesian_plan(const std::shared_ptr<xarm_msgs::srv::PlanSingleStraight::Request> req, std::shared_ptr<xarm_msgs::srv::PlanSingleStraight::Response> res);
	bool exec_plan_cb(const std::shared_ptr<xarm_msgs::srv::PlanExec::Request> req, std::shared_ptr<xarm_msgs::srv::PlanExec::Response> res);
	void stop_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

private:
	rclcpp::Node::SharedPtr node_;
	std::shared_ptr<xarm_planner::XArmPlanner> xarm_planner_;

	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exec_plan_sub_;

	rclcpp::Service<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_server_;
	rclcpp::Service<xarm_msgs::srv::PlanPose>::SharedPtr pose_plan_server_;
	rclcpp::Service<xarm_msgs::srv::PlanPoseWeighted>::SharedPtr pose_plan_weighted_server_;
	rclcpp::Service<xarm_msgs::srv::PlanJoint>::SharedPtr joint_plan_server_;
	rclcpp::Service<xarm_msgs::srv::PlanSingleStraight>::SharedPtr single_straight_plan_server_;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_server_;

	rclcpp::CallbackGroup::SharedPtr exec_group_;
	rclcpp::CallbackGroup::SharedPtr cancel_group_;
};

XArmPlannerRunner::XArmPlannerRunner(rclcpp::Node::SharedPtr& node)
	: node_(node)
{
	int dof;
	node_->get_parameter_or("dof", dof, 7);
	std::string robot_type;
	node_->get_parameter_or("robot_type", robot_type, std::string("xarm"));
	std::string group_name = robot_type;
	if (robot_type == "xarm" || robot_type == "lite")
		group_name = robot_type + std::to_string(dof);
	std::string prefix;
	node->get_parameter_or("prefix", prefix, std::string(""));
	if (prefix != "") {
		group_name = prefix + group_name;
	}

	RCLCPP_INFO(node_->get_logger(), "namespace: %s, group_name: %s", node->get_namespace(), group_name.c_str());

	xarm_planner_ = std::make_shared<xarm_planner::XArmPlanner>(node_, group_name);

	setup_servers();
}

void XArmPlannerRunner::setup_servers()
{
	exec_group_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	cancel_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	exec_plan_server_ = node_->create_service<xarm_msgs::srv::PlanExec>(
	                        "xarm_exec_plan",
	                        BIND_CLS_CB(&XArmPlannerRunner::exec_plan_cb),
	                        rmw_qos_profile_services_default,
	                        exec_group_);

	pose_plan_server_ = node_->create_service<xarm_msgs::srv::PlanPose>(
	                        "xarm_pose_plan",
	                        BIND_CLS_CB(&XArmPlannerRunner::do_pose_plan),
	                        rmw_qos_profile_services_default,
	                        exec_group_);

	pose_plan_weighted_server_ = node_->create_service<xarm_msgs::srv::PlanPoseWeighted>(
	                                 "xarm_pose_plan_weighted",
	                                 BIND_CLS_CB(&XArmPlannerRunner::do_pose_plan_weighted),
	                                 rmw_qos_profile_services_default,
	                                 exec_group_);

	joint_plan_server_ = node_->create_service<xarm_msgs::srv::PlanJoint>(
	                         "xarm_joint_plan",
	                         BIND_CLS_CB(&XArmPlannerRunner::do_joint_plan),
	                         rmw_qos_profile_services_default,
	                         exec_group_);

	single_straight_plan_server_ = node_->create_service<xarm_msgs::srv::PlanSingleStraight>(
	                                   "xarm_straight_plan",
	                                   BIND_CLS_CB(&XArmPlannerRunner::do_single_cartesian_plan),
	                                   rmw_qos_profile_services_default,
	                                   exec_group_);

	stop_server_ = node_->create_service<std_srvs::srv::Trigger>(
	                   "xarm_stop",
	                   BIND_CLS_CB(&XArmPlannerRunner::stop_cb),
	                   rmw_qos_profile_services_default,
	                   cancel_group_);
}

bool XArmPlannerRunner::do_pose_plan(const std::shared_ptr<xarm_msgs::srv::PlanPose::Request> req, std::shared_ptr<xarm_msgs::srv::PlanPose::Response> res)
{
	bool success = xarm_planner_->planPoseTarget(req->target);
	res->success = success;
	return success;
}

bool XArmPlannerRunner::do_pose_plan_weighted(
    const std::shared_ptr<xarm_msgs::srv::PlanPoseWeighted::Request> req,
    std::shared_ptr<xarm_msgs::srv::PlanPoseWeighted::Response> res)
{
	double cost = -1.0;
	bool success = xarm_planner_->planPoseTarget(req->target, cost);

	res->success = success;
	res->cost = cost;

	return success;
}

bool XArmPlannerRunner::do_joint_plan(const std::shared_ptr<xarm_msgs::srv::PlanJoint::Request> req, std::shared_ptr<xarm_msgs::srv::PlanJoint::Response> res)
{
	bool success = xarm_planner_->planJointTarget(req->target);
	res->success = success;
	return success;
}

bool XArmPlannerRunner::do_single_cartesian_plan(const std::shared_ptr<xarm_msgs::srv::PlanSingleStraight::Request> req, std::shared_ptr<xarm_msgs::srv::PlanSingleStraight::Response> res)
{
	std::vector<geometry_msgs::msg::Pose> waypoints;
	waypoints.push_back(req->target);
	bool success = xarm_planner_->planCartesianPath(waypoints);
	res->success = success;
	return success;
}

bool XArmPlannerRunner::exec_plan_cb(const std::shared_ptr<xarm_msgs::srv::PlanExec::Request> req, std::shared_ptr<xarm_msgs::srv::PlanExec::Response> res)
{
	bool success = xarm_planner_->executePath(req->wait);
	res->success = success;
	return success;
}

void XArmPlannerRunner::stop_cb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
	xarm_planner_->stop();
	res->success = true;
	res->message = "Stop sent to MoveIt";
}

void exit_sig_handler(int signum)
{
	fprintf(stderr, "[xarm_planner_node] Ctrl-C caught, exit process...\n");
	exit(-1);
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_planner_node", node_options);
	RCLCPP_INFO(node->get_logger(), "xarm_planner_node start");
	signal(SIGINT, exit_sig_handler);

	XArmPlannerRunner xarm_planner_runner(node);

	rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
	exec.add_node(node);
	exec.spin();
	rclcpp::shutdown();

	RCLCPP_INFO(node->get_logger(), "xarm_planner_node over");
	return 0;
}