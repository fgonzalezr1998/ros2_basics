#include <rclcpp/rclcpp.hpp>

#include <octomap_msgs/msg/octomap.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2/exceptions.h>
#include <tf2/convert.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
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
        octomap::OcTree *erosion_octree;
        octomap::ColorOcTree *kdtree_octree;

        kdtree_octree = dynamic_cast<octomap::ColorOcTree*>(
            octomap_msgs::fullMsgToMap(*kdtree_octomap_));
        erosion_octree = dynamic_cast<octomap::OcTree*>(
            octomap_msgs::fullMsgToMap(*erosion_octomap_));

        if (kdtree_octree == NULL || erosion_octree == NULL) {
            RCLCPP_ERROR(this->get_logger(), "Message to Map conversion failed!\n");
            return;
        }

        if (kdtree_octree->getResolution() != erosion_octree->getResolution()) {
            RCLCPP_ERROR(this->get_logger(), "Resolutions are not equals!");
            return;
        }

        octree_aux_ = new octomap::OcTree(kdtree_octomap_->resolution);

        if (!transform_octree(erosion_octree)) {
            return;
        }

        int corrects, total;
        corrects = 0;
        total = 0;
        for (octomap::ColorOcTree::leaf_iterator it = kdtree_octree->begin_leafs(),
            end = kdtree_octree->end_leafs(); it != end; it++)
        {
            if (node_exists(erosion_octree, it)) {
                corrects++;
            }
            total++;
        }

        free(kdtree_octree);
        free(erosion_octree);
        free(octree_aux_);

        RCLCPP_INFO(get_logger(), "Corrects: %d / %d", corrects, total);
        RCLCPP_INFO(get_logger(), "Percentage: %f\n", (float)corrects * 100.0 / (float)total);
    }

private:
    bool
    node_exists(octomap::OcTree * erosion_octree, octomap::ColorOcTree::leaf_iterator it)
    {
        double res;

        res = erosion_octree->getResolution();

        //RCLCPP_INFO(get_logger(), "Procesando!");
        for (octomap::OcTree::leaf_iterator i = erosion_octree->begin_leafs(),
            end = erosion_octree->end_leafs(); i != end; i++)
        {
            if (std::fabs(it.getX() - i.getX()) <= res / 4.0 ||
                std::fabs(it.getY() - i.getY()) <= res / 4.0 ||
                std::fabs(it.getZ() - i.getZ()) <= res / 4.0)
            {
                return true;
            }
        }

        return false;
    }

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
        geometry_msgs::msg::PointStamped ps, ps_out;

        for (octomap::OcTree::leaf_iterator it = erosion_octree->begin_leafs(),
            end = erosion_octree->end_leafs(); it != end; it++)
        {
            p3d = it.getCoordinate();

            ps.header.frame_id = erosion_octomap_->header.frame_id;
            ps.header.stamp = this->now();
            ps.point.x = p3d.x(); ps.point.y = p3d.y(); ps.point.z = p3d.z();

            // transform the point
            try{
                transformStamped = tf_buffer_.lookupTransform(kdtree_octomap_->header.frame_id,
                    erosion_octomap_->header.frame_id, rclcpp::Time(0));
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(get_logger(), "%s", ex.what());
                return false;
            }

            tf2::doTransform(ps, ps_out, transformStamped);


            octree_aux_->setNodeValue(ps_out.point.x, ps_out.point.y,
                ps_out.point.z, it->getValue());

            octree_aux_->updateInnerOccupancy();
        }

        // erosion_octree->clear();
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
        this->declare_parameter("octomap_kdtree_topic",
            "yolact_ros2_3d_node_octomaps/output_octomaps");
        this->declare_parameter("octomap_erosion_topic",
            "yolact_ros2_3d/octomaps/dynamics/person");

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

    rclcpp::Rate loop_rate(20);
    while(rclcpp::ok()) {
        comparator_node->step();
        rclcpp::spin_some(comparator_node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    exit(EXIT_SUCCESS);
}