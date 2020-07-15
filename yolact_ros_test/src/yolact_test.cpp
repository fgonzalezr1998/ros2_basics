#include <rclcpp/rclcpp.hpp>
#include "yolact_ros_msgs/msg/detections.hpp"
#include "yolact_ros_msgs/msg/detection.hpp"

using std::placeholders::_1;

class YolactTest: public rclcpp::Node
{
public:

	YolactTest(std::string node_name):
		Node(node_name)
	{
		RCLCPP_INFO(this->get_logger(), "Hi! I am '%s' node\n", this->get_name());

		yolact_dets_sub_ = this->create_subscription<yolact_ros_msgs::msg::Detections>("/yolact_ros/detections",
											1, std::bind(&YolactTest::yolact_dets_cb, this, _1));
	}

	void
	step()
	{
		RCLCPP_INFO(this->get_logger(), "%s\n", "Step!");
		for(auto det : detections_received_)
		{
			if(det.class_name != "person")
				continue;
			this->process_detection(det);
		}
	}

private:

	void
	yolact_dets_cb(const yolact_ros_msgs::msg::Detections::SharedPtr msg)
	{
		detections_received_ = msg->detections;
	}

	void
	process_detection(yolact_ros_msgs::msg::Detection detection)
	{
		;
	}

	rclcpp::Subscription<yolact_ros_msgs::msg::Detections>::SharedPtr yolact_dets_sub_;
	std::vector<yolact_ros_msgs::msg::Detection>detections_received_;
};

int
main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	auto yolact_test_node = std::make_shared<YolactTest>("yolact_test_node");

	rclcpp::Rate loop_rate(10);
	while(rclcpp::ok())
	{
		yolact_test_node->step();
		rclcpp::spin_some(yolact_test_node);
		loop_rate.sleep();
	}

	rclcpp::shutdown();
	exit(EXIT_SUCCESS);
}
