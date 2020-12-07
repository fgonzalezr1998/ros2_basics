#include <rclcpp/rclcpp.hpp>

#include <octomap_msgs/msg/octomap.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "octomap/octomap.h"
#include "octomap/OcTree.h"
#include "octomap/OcTreeKey.h"
#include "octomap_msgs/conversions.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <string>

using std::placeholders::_1;

class Comparator : public rclcpp::Node
{
public:
    Comparator(const std::string & node_name)
    :
    Node(node_name), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
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

        octree_aux_ = new octomap::OcTree(kdtree_octomap_->resolution);

        if (!transform_octree(erosion_octree)) {
            return;
        }

        octomap::OcTreeKey key;
        octomap::OcTreeNode *node = NULL;
        int corrects, total;
        corrects = 0;
        total = 0;
        for (octomap::OcTree::leaf_iterator it = kdtree_octree->begin_leafs(),
            end = kdtree_octree->end_leafs(); it != end; it++)
        {
            node = erosion_octree->search(it.getKey());
            if (node != NULL) {
                corrects++;
            }
            total++;
        }

        free(kdtree_octree);
        free(erosion_octree);
        free(octree_aux_);

        RCLCPP_INFO(get_logger(), "%s\n", octomap_kdtee_topic_);
        RCLCPP_INFO(get_logger(), "%s\n", octomap_erosion_topic_);

        RCLCPP_INFO(get_logger(), "Corrects: %d\n", corrects);
    }

private:
    bool
    transform_octree(octomap::OcTree * erosion_octree)
    {
        /*
         * If frame are different, convert 'erosion_octomap_' to the same frame of 'kdtree_octomap_'
         */

        if (kdtree_octomap_->header.frame_id == erosion_octomap_->header.frame_id) {
            return true;
        }

        octomap::point3d p3d;
        geometry_msgs::msg::TransformStamped transformStamped;
        geometry_msgs::msg::Point point;

        for (octomap::OcTree::leaf_iterator it = erosion_octree->begin_leafs(),
            end = erosion_octree->end_leafs(); it != end; it++)
        {
            p3d = it.getCoordinate();

            // transform the point
            try{
                transformStamped = tf_buffer_.lookupTransform(kdtree_octomap_->header.frame_id,
                    erosion_octomap_->header.frame_id, rclcpp::Time(0));
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(get_logger(), "%s", ex.what());
                return false;
            }

            point.x = (double)p3d.x() + transformStamped.transform.translation.x;
            point.y = (double)p3d.y() + transformStamped.transform.translation.y;
            point.z = (double)p3d.z() + transformStamped.transform.translation.z;

            octree_aux_->setNodeValue(point.x, point.y, point.z, it->getValue());
        }

        erosion_octree->clear();
        erosion_octree = octree_aux_;

        return true;
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
    octomap::OcTree *octree_aux_;

    std::string octomap_kdtee_topic_, octomap_erosion_topic_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
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