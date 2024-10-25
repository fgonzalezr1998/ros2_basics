#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>
#include <string>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

#define LOOP_RATE   1.0   // Hz

class TimerNodeCpp :  public rclcpp::Node
{
public:
    TimerNodeCpp(const std::string & node_name)
    : Node(node_name), i_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hi!, I'm the [%s] node\n", node_name.c_str());
        this->create_wall_timer(
            500ms,
            std::bind(&TimerNodeCpp::step, this));
    }

private:
    void step()
    {
        RCLCPP_INFO(this->get_logger(), "Step [%d]\n", i_++);
    }

    int i_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<TimerNodeCpp> iterative_node = std::make_shared<TimerNodeCpp>("timer_node_cpp");

    rclcpp::spin(iterative_node);

    rclcpp::shutdown();
    exit(EXIT_SUCCESS);
}