#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <string>

using std::placeholders::_1;

class TFTransformer: public rclcpp::Node
{
public:
	TFTransformer(std::string node_name):
		Node(node_name), pc_received_(false), clock_(RCL_SYSTEM_TIME),
    tfBuffer_(std::make_shared<rclcpp::Clock>(clock_)), tfListener_(tfBuffer_, true)
	{
		pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/pointcloud", 1, std::bind(&TFTransformer::pointcloudCb, this, _1));
	}

	void
	step()
	{
		geometry_msgs::msg::TransformStamped transform;
		sensor_msgs::msg::PointCloud2 cloud_out;
		sensor_msgs::msg::PointCloud cloud_pc;

		if(! pc_received_)
			return;

		try
    {
      transform = tfBuffer_.lookupTransform("camera_link", original_cloud_.header.frame_id,
          original_cloud_.header.stamp, tf2::durationFromSec(1.0));

    }
    catch(tf2::TransformException& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Transform error of sensor data: %s, %s\n", ex.what(), "quitting callback");
      return;
    }
    tf2::doTransform<sensor_msgs::msg::PointCloud2>(original_cloud_, cloud_out, transform);
		sensor_msgs::convertPointCloud2ToPointCloud(cloud_out, cloud_pc);

		RCLCPP_INFO(this->get_logger(), "%s\n", "Everything was OK!");
	}

private:

	void
	pointcloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
	{
		original_cloud_ = *msg;
		pc_received_ = true;
	}


	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
	sensor_msgs::msg::PointCloud2 original_cloud_;
	bool pc_received_;
	rclcpp::Clock clock_;
	tf2_ros::Buffer tfBuffer_;
	tf2_ros::TransformListener tfListener_;
};

int
main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	//Init node
	auto transformer = std::make_shared<TFTransformer>("pointcloud_tf_transformer_node");

	rclcpp::Rate loop_rate(5);	//5Hz
	while(rclcpp::ok())
	{
		rclcpp::spin_some(transformer->get_node_base_interface());
		transformer->step();
		loop_rate.sleep();
	}

	rclcpp::shutdown();

	exit(EXIT_SUCCESS);
}
