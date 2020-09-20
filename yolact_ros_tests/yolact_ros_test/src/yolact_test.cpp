#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "yolact_ros2_msgs/msg/detections.hpp"
#include "yolact_ros2_msgs/msg/detection.hpp"
#include "yolact_ros2_msgs/msg/mask.hpp"

using std::placeholders::_1;

class YolactTest: public rclcpp::Node
{
public:

	YolactTest(std::string node_name):
		Node(node_name)
	{
		RCLCPP_INFO(this->get_logger(), "Hi! I am '%s' node\n", this->get_name());

		yolact_dets_sub_ = this->create_subscription<yolact_ros2_msgs::msg::Detections>("/yolact_ros2/detections",
											1, std::bind(&YolactTest::yolact_dets_cb, this, _1));
		camera_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw",
											1, std::bind(&YolactTest::camera_img_cb_, this, _1));
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
	yolact_dets_cb(const yolact_ros2_msgs::msg::Detections::SharedPtr msg)
	{
		detections_received_ = msg->detections;
	}

	void
	camera_img_cb_(const sensor_msgs::msg::Image::SharedPtr msg)
	{
		img_received_ = *msg;
	}

	void
	process_detection(yolact_ros2_msgs::msg::Detection detection)
	{
		cv::Mat mask;

		mask = cv::Mat(detection.mask.height, detection.mask.width, CV_8U);
		for(int x = 0; x < detection.mask.width; x++)
		{
			for(int y = 0; y < detection.mask.height; y++)
			{
				if(this->test(detection.mask, x, y))
					mask.at<unsigned char>(y, x) = 255;
				else
					mask.at<unsigned char>(y, x) = 0;
			}
		}

		//Erode the image with 3x3 kernel

		cv::Mat mask_eroded;
    cv::erode(mask, mask_eroded, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(50, 50)));

		cv::imshow("Original Mask", mask);
		cv::waitKey(1);
		cv::imshow("Eroded Mask", mask_eroded);
		cv::waitKey(1);
	}

	bool
	test(const yolact_ros2_msgs::msg::Mask &mask, size_t x, size_t y)
  {
		size_t index, byte_ind, bit_ind;

    index = y * mask.width + x;
    byte_ind = index / 8;
    bit_ind = 7 - (index % 8); // bitorder 'big'
    return mask.mask[byte_ind] & (1 << bit_ind);
  }

	rclcpp::Subscription<yolact_ros2_msgs::msg::Detections>::SharedPtr yolact_dets_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_img_sub_;

	std::vector<yolact_ros2_msgs::msg::Detection>detections_received_;
	sensor_msgs::msg::Image img_received_;
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
