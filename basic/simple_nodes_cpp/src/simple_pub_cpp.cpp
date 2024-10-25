#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>
#include <string>
#include <memory>

#define LOOP_RATE   1.0   // Hz

class SimplePub :  public rclcpp::Node
{
public:
    SimplePub(const std::string & node_name)
    : Node(node_name), i_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hi!, I'm the [%s] node\n", node_name.c_str());
    }

    void step()
    {
        RCLCPP_INFO(this->get_logger(), "Step [%d]\n", i_++);
    }
private:
    int i_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto simple_pub_node = std::make_shared<SimplePub>("simple_pub_node_cpp");

    rclcpp::Rate loop_rate(LOOP_RATE);

    while (rclcpp::ok()) {
        simple_pub_node->step();
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    exit(EXIT_SUCCESS);
}