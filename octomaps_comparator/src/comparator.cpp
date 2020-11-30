#include <rclcpp/rclcpp.hpp>
#include <string>

class Comparator : public rclcpp::Node
{
public:

    Comparator(const std::string node_name)
    :
    Node(node_name)
    {
        ;
    }

private:
};

int
main(int argc, char ** argv)
{
    Comparator::SharedPtr comparator_node;

    rclcpp::init(argc, argv);

    comparator_node = std::make_shared<Comparator>("octomaps_comparator_node");

    exit(EXIT_SUCCESS);
}