// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/jetracer_order.hpp"
#include "control_msgs/msg/jetracer_order.hpp"

using std::placeholders::_1;

class VelocityNode : public rclcpp::Node
{
	public:
	VelocityNode() : Node("jetracer_velocity_selector")
	{
		publisher_ = this->create_publisher<control_msgs::msg::JetracerOrder>("/jetracer_order",10);
		subscription_ = this->create_subscription<control_msgs::msg::JetracerOrder>("/jetracer_order_logi", 10, std::bind(&VelocityNode::jetracer_order_callback, this, _1));
	}
	private:
	rclcpp::Publisher<control_msgs::msg::JetracerOrder>::SharedPtr publisher_;
	rclcpp::Subscription<control_msgs::msg::JetracerOrder>::SharedPtr subscription_;

	void jetracer_order_callback(const control_msgs::msg::JetracerOrder::ConstPtr msg) {
		control_msgs::msg::JetracerOrder jetracer_order;
			jetracer_order.header = msg->header;
			jetracer_order.enable = msg->enable;
			jetracer_order.set_vel = msg->set_vel;
			jetracer_order.str_angle = msg->str_angle;
			jetracer_order.str_offset = msg->str_offset;
		publisher_->publish(jetracer_order);
	}
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VelocityNode>());
	rclcpp::shutdown();
	return 0;
}
