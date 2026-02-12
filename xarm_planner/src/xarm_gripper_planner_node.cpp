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

	bool do_joint_plan(const std::shared_ptr<xarm_msgs::srv::PlanJoint::Request> req, std::shared_ptr<xarm_msgs::srv::PlanJoint::Response> res);
	bool exec_plan_cb(const std::shared_ptr<xarm_msgs::srv::PlanExec::Request> req, std::shared_ptr<xarm_msgs::srv::PlanExec::Response> res);
	void stop_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

private:
	rclcpp::Node::SharedPtr node_;
	std::shared_ptr<xarm_planner::XArmPlanner> xarm_planner_;

	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exec_plan_sub_;

	rclcpp::Service<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_server_;
	rclcpp::Service<xarm_msgs::srv::PlanJoint>::SharedPtr joint_plan_server_;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_server_;

	rclcpp::CallbackGroup::SharedPtr exec_group_;
	rclcpp::CallbackGroup::SharedPtr cancel_group_;
};

void XArmPlannerRunner::setup_servers()
{
	exec_group_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	cancel_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	exec_plan_server_ = node_->create_service<xarm_msgs::srv::PlanExec>(
	                        "xarm_gripper_exec_plan",
	                        BIND_CLS_CB(&XArmPlannerRunner::exec_plan_cb),
	                        rmw_qos_profile_services_default,
	                        exec_group_);

	joint_plan_server_ = node_->create_service<xarm_msgs::srv::PlanJoint>(
	                         "xarm_gripper_joint_plan",
	                         BIND_CLS_CB(&XArmPlannerRunner::do_joint_plan),
	                         rmw_qos_profile_services_default,
	                         exec_group_);

	stop_server_ = node_->create_service<std_srvs::srv::Trigger>(
	                   "xarm_gripper_stop",
	                   BIND_CLS_CB(&XArmPlannerRunner::stop_cb),
	                   rmw_qos_profile_services_default,
	                   cancel_group_);
}

XArmPlannerRunner::XArmPlannerRunner(rclcpp::Node::SharedPtr& node)
	: node_(node)
{
	std::string group_name;
	node_->get_parameter_or("PLANNING_GROUP", group_name, std::string("xarm_gripper"));

	RCLCPP_INFO(node_->get_logger(), "namespace: %s, group_name: %s",
	            node->get_namespace(), group_name.c_str());

	xarm_planner_ = std::make_shared<xarm_planner::XArmPlanner>(node_, group_name);

	setup_servers();
}

bool XArmPlannerRunner::do_joint_plan(const std::shared_ptr<xarm_msgs::srv::PlanJoint::Request> req, std::shared_ptr<xarm_msgs::srv::PlanJoint::Response> res)
{
	bool success = xarm_planner_->planJointTarget(req->target);
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
	res->message = "Stop sent to MoveIt (gripper)";
}

void exit_sig_handler(int signum)
{
	fprintf(stderr, "[xarm_gripper_planner_node] Ctrl-C caught, exit process...\n");
	exit(-1);
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("xarm_gripper_planner_node", node_options);
	RCLCPP_INFO(node->get_logger(), "xarm_gripper_planner_node start");
	signal(SIGINT, exit_sig_handler);

	XArmPlannerRunner xarm_planner_runner(node);
	rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
	exec.add_node(node);
	exec.spin();
	rclcpp::shutdown();

	RCLCPP_INFO(node->get_logger(), "xarm_gripper_planner_node over");
	return 0;
}