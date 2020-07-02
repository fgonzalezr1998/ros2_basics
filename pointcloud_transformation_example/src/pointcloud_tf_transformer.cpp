#include <rclcpp/rclcpp.hpp>
#include <string>

class TFTransformer: public rclcpp::Node
{
public:
	TFTransformer(std::string node_name):
		Node(node_name)
	{
		RCLCPP_INFO(this->get_logger(), "%s\n", "Hello World!");
		//pointcloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &TFTransformer::pointcloudCb, this);
	}

	void
	step()
	{
		/*
		geometry_msgs::TransformStamped transform;
		sensor_msgs::PointCloud2 cloud_out;

		if(! pc_received_)
			return;

		try
		{
			transform = tf_buffer_.lookupTransform("camera_link", original_cloud_.header.frame_id,
	        				original_cloud_.header.stamp + ros::Duration(2.0), ros::Duration(2.0));

			tf2::doTransform(original_cloud_, cloud_out, transform);
		}catch(tf2::TransformException& ex){
			ROS_WARN("%s", ex.what());
      return;
		}
		*/
		RCLCPP_INFO(this->get_logger(), "%s\n", "Everything was OK!");
	}

private:
	/*
	void
	pointcloudCb(const sensor_msgs::PointCloud2::ConstPtr &msg)
	{
		original_cloud_ = *msg;
		pc_received_ = true;
	}

	ros::NodeHandle nh_;
	ros::Subscriber pointcloud_sub_;
	tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
	sensor_msgs::PointCloud2 original_cloud_;
	bool pc_received_;
	*/
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
