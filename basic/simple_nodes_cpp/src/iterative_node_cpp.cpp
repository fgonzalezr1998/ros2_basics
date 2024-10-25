#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>
#include <string>
#include <memory>

#define LOOP_RATE   1.0   // Hz

class ItNodeCpp :  public rclcpp::Node
{
public:
    ItNodeCpp(const std::string & node_name)
    : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "Hi!, I'm the [%s] node\n", node_name.c_str());
    }

    void step()
    {
        RCLCPP_INFO(this->get_logger(), "Step\n");
    }
private:

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // std::shared_ptr<ItNodeCpp> iteratve_node = std::make_shared<ItNodeCpp>("terative_node_cpp");
    // ItNodeCpp *iterative_node = new ItNodeCpp("terative_node_cpp");
    ItNodeCpp iterative_node("terative_node_cpp");

    rclcpp::Rate loop_rate(LOOP_RATE);

    while (rclcpp::ok()) {
        iterative_node.step();
        loop_rate.sleep();
    }

    exit(EXIT_SUCCESS);
}