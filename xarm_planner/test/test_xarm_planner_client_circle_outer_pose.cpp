/* test_xarm_circle_outer_pose.cpp
 *
 * Move the TCP around a full 360° circle on the OUTER radius only.
 * XY follows a circle with radius=0.71 (your measured max at this Z+orientation),
 * stepping by 50 degrees. Z and orientation stay fixed.
 *
 * Services used:
 *  - xarm_pose_plan
 *  - xarm_exec_plan
 */

#include <signal.h>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include <xarm_msgs/srv/plan_pose.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>

#define SERVICE_CALL_FAILED 999

std::shared_ptr<rclcpp::Node> node;

void exit_sig_handler(int /*signum*/)
{
	fprintf(stderr, "[test_xarm_circle_outer_pose] Ctrl-C caught, exit process...\n");
	exit(-1);
}

template<typename SrvT>
int call_request(typename rclcpp::Client<SrvT>::SharedPtr client,
                 typename SrvT::Request::SharedPtr req)
{
	bool is_try_again = false;
	while (!client->wait_for_service(std::chrono::seconds(1))) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
			exit(1);
		}
		if (!is_try_again) {
			is_try_again = true;
			RCLCPP_WARN(node->get_logger(), "service %s not available, waiting ...", client->get_service_name());
		}
	}

	auto result_future = client->async_send_request(req);
	if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
		RCLCPP_ERROR(node->get_logger(), "Failed to call service %s", client->get_service_name());
		return SERVICE_CALL_FAILED;
	}

	auto res = result_future.get();
	RCLCPP_INFO(node->get_logger(), "call service %s, success=%d", client->get_service_name(), res->success);
	return res->success;
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);

	node = rclcpp::Node::make_shared("test_xarm_circle_outer_pose", node_options);
	RCLCPP_INFO(node->get_logger(), "test_xarm_circle_outer_pose start");
	signal(SIGINT, exit_sig_handler);

	// Params (override if you want)
	double radius_m = 0.715;   // your max reach radius at this Z+orientation
	double z_m      = 0.175;   // keep unchanged
	double step_deg = 1.0;   // 360 in jumps of 1 deg

	node->get_parameter_or("radius_m", radius_m, radius_m);
	node->get_parameter_or("z_m",      z_m,      z_m);
	node->get_parameter_or("step_deg", step_deg, step_deg);

	RCLCPP_INFO(node->get_logger(), "namespace: %s", node->get_namespace());
	RCLCPP_INFO(node->get_logger(), "circle params: radius=%.3f z=%.3f step_deg=%.1f", radius_m, z_m, step_deg);

	// Services
	auto pose_plan_client_ = node->create_client<xarm_msgs::srv::PlanPose>("xarm_pose_plan");
	auto exec_plan_client_ = node->create_client<xarm_msgs::srv::PlanExec>("xarm_exec_plan");

	auto pose_plan_req = std::make_shared<xarm_msgs::srv::PlanPose::Request>();
	auto exec_plan_req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();
	exec_plan_req->wait = true;

	// Fixed pose template (orientation + Z stay constant)
	geometry_msgs::msg::Pose target_pose;
	target_pose.position.z = z_m;

	// Keep unchanged (as you requested)
	target_pose.orientation.x = 1.0;
	target_pose.orientation.y = 0.0;
	target_pose.orientation.z = 0.0;
	target_pose.orientation.w = 0.0;

	constexpr double kPi = 3.14159265358979323846;

	while (rclcpp::ok())
	{
		// Outer circle only: 0..360 in steps of step_deg
		// (exclude 360 to avoid duplicating the 0-degree point)
		for (double deg = 0.0; rclcpp::ok() && deg < 360.0; deg += step_deg)
		{
			const double rad = deg * kPi / 180.0;

			target_pose.position.x = radius_m * std::cos(rad);
			target_pose.position.y = radius_m * std::sin(rad);

			RCLCPP_INFO(node->get_logger(), "Target deg=%.1f -> x=%.3f y=%.3f z=%.3f",
			            deg, target_pose.position.x, target_pose.position.y, target_pose.position.z);

			pose_plan_req->target = target_pose;

			int ok = call_request<xarm_msgs::srv::PlanPose>(pose_plan_client_, pose_plan_req);
			if (ok != 1) {
				RCLCPP_ERROR(node->get_logger(), "Planning failed at deg=%.1f (x=%.3f y=%.3f). Stopping loop.",
				             deg, target_pose.position.x, target_pose.position.y);
				break;
			}

			ok = call_request<xarm_msgs::srv::PlanExec>(exec_plan_client_, exec_plan_req);
			if (ok != 1) {
				RCLCPP_ERROR(node->get_logger(), "Execution failed at deg=%.1f (x=%.3f y=%.3f). Stopping loop.",
				             deg, target_pose.position.x, target_pose.position.y);
				break;
			}
		}
	}

	RCLCPP_INFO(node->get_logger(), "test_xarm_circle_outer_pose over");
	rclcpp::shutdown();
	return 0;
}
