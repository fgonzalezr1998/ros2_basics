#include <rclcpp/rclcpp.hpp>

int
main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	//Init node
  auto node = rclcpp::Node::make_shared("pointcloud_tf_transformer_node");

	rclcpp::shutdown();

	exit(EXIT_SUCCESS);
}
