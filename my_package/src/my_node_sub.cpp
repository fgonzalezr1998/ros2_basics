#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MySubscriber
{
public:

    MySubscriber(rclcpp::Node::SharedPtr node)
    {
        this->node_ = node;
        sub_ = this->node_->create_subscription<std_msgs::msg::String>("/talker", 1, std::bind(&MySubscriber::callback, this, std::placeholders::_1));
    }

private:

    void
    callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->node_->get_logger(), "I heard: [%s]", msg->data.c_str());
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

};

int
main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("my_node_sub");

    MySubscriber sub(node);

    rclcpp::spin(node);

    rclcpp::shutdown();

    exit(EXIT_SUCCESS);
}
