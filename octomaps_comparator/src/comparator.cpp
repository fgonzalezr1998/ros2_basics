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

        kdtree_octomap_ = nullptr;
        erosion_octomap_ = nullptr;

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
        if (kdtree_octomap_ == nullptr || erosion_octomap_ == nullptr) {
            return;
        }

        octomap::OcTree *kdtree_octree, *erosion_octree;

        kdtree_octree = dynamic_cast<octomap::OcTree*>(
            octomap_msgs::fullMsgToMap(*kdtree_octomap_));
        erosion_octree = dynamic_cast<octomap::OcTree*>(
            octomap_msgs::fullMsgToMap(*erosion_octomap_));

        if (kdtree_octree->getResolution() != erosion_octree->getResolution()) {
            RCLCPP_ERROR(this->get_logger(), "Resolutions are not equals!");
            return;
        }

        octomap::OcTreeKey key;

        transform_octree(erosion_octree);

        for (octomap::OcTree::leaf_iterator it = kdtree_octree->begin_leafs(),
            end = kdtree_octree->end_leafs(); it != end; it++)
        {
            ;
        }
    }

private:
    void
    transform_octree(octomap::OcTree * erosion_octree)
    {
        /*
         * If frame are different, convert 'erosion_octomap_' to the same frame of 'kdtree_octomap_'
         */

        if (kdtree_octomap_->header.frame_id == erosion_octomap_->header.frame_id) {
            return;
        }

        octomap::point3d p3d;
        for (octomap::OcTree::leaf_iterator it = erosion_octree->begin_leafs(),
            end = erosion_octree->end_leafs(); it != end; it++)
        {
            p3d = it.getCoordinate();

            // transform the point

        }
    }

    void
    octomap_kdtreeCb(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        kdtree_octomap_ = msg;
    }

    void
    octomap_erosionCb(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        erosion_octomap_= msg;
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

    octomap_msgs::msg::Octomap::SharedPtr kdtree_octomap_, erosion_octomap_;

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