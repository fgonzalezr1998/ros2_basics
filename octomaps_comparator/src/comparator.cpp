#include <string>
#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>

using std::placeholders::_1;

class Comparator : public rclcpp::Node
{
public:
    Comparator(const std::string & node_name)
    :
    Node(node_name)
    {
        initParams();

        octomap_kdtree_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            octomap_kdtee_topic_,
            rclcpp::QoS(1).best_effort().durability_volatile(),
            std::bind(&Comparator::octomap_kdtreeCb, this, std::placeholders::_1));

        octomap_erosion_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            octomap_erosion_topic_,
            rclcpp::QoS(1).best_effort().durability_volatile(),
            std::bind(&Comparator::octomap_erosionCb, this, std::placeholders::_1));
    }

    void
    step()
    {
        ;
    }

private:
    void
    octomap_kdtreeCb(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        kdtree_octree_ = msg;
    }

    void
    octomap_erosionCb(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        erosion_octree_ = msg;
    }

    void
    initParams()
    {
        this->declare_parameter("octomap_kdtree_topic", "octomap_kdtree");
        this->declare_parameter("octomap_erosion_topic", "octomap_erosion");

        this->get_parameter("octomap_kdtree_topic", octomap_kdtee_topic_);
        this->get_parameter("octomap_erosion_topic", octomap_erosion_topic_);
    }

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_kdtree_sub_,
        octomap_erosion_sub_;

    octomap_msgs::msg::Octomap::SharedPtr kdtree_octree_, erosion_octree_;

    std::string octomap_kdtee_topic_, octomap_erosion_topic_;
};

int
main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto comparator_node = std::make_shared<Comparator>("octomaps_comparator_node");

    rclcpp::Rate loop_rate(10);
    while(rclcpp::ok()) {
        comparator_node->step();
        rclcpp::spin_some(comparator_node);
        loop_rate.sleep();
    }

    exit(EXIT_SUCCESS);
}