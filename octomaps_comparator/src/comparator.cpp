#include <rclcpp/rclcpp.hpp>

#include <octomap_msgs/msg/octomap.hpp>
#include "octomap/octomap.h"
#include "octomap/OcTree.h"
#include "octomap/OcTreeKey.h"
#include "octomap_msgs/conversions.h"

#include <string>

using std::placeholders::_1;

class Comparator : public rclcpp::Node
{
public:
    Comparator(const std::string & node_name)
    :
    Node(node_name)
    {
        initParams();

        kdtree_octree_ = nullptr;
        erosion_octree_ = nullptr;

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
        if (kdtree_octree_ == nullptr || erosion_octree_ == nullptr) {
            return;
        }

        if (kdtree_octree_->getResolution() != erosion_octree_->getResolution()) {
            RCLCPP_ERROR(this->get_logger(), "Resolutions are not equals!");
            return;
        }

        for (octomap::OcTree::leaf_iterator it = kdtree_octree_->begin_leafs(),
            end = kdtree_octree_->end_leafs(); it != end; it++)
        {
            ;
        }
    }

private:
    void
    octomap_kdtreeCb(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        kdtree_octree_ = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg));
    }

    void
    octomap_erosionCb(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        erosion_octree_ = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg));
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

    octomap::OcTree *kdtree_octree_, *erosion_octree_;

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