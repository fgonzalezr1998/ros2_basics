// Copyright 2020
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Author: Fernando González fergonzaramos@yahoo.es */


#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include "gb_visual_detection_3d_msgs/msg/bounding_boxes3d.hpp"
#include "gb_visual_detection_3d_msgs/msg/bounding_box3d.hpp"

#define HZ 5

using std::placeholders::_1;

class Bboxes3d2nav2 : public rclcpp::Node
{
public:
  Bboxes3d2nav2(const std::string & node_name)
  : Node(node_name), person_saw_(false)
  {
    initParams();

    yolact_sub_ = this->create_subscription
      <gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>(
      yolact_topic_, 1, std::bind(&Bboxes3d2nav2::yolactCallback, this, _1));
  }

  void
  step()
  {
    bool person_found;
    gb_visual_detection_3d_msgs::msg::BoundingBox3d bbox_detected;

    if (newPerson(person_found, bbox_detected)) {
      person_saw_ = true;
      RCLCPP_INFO(get_logger(), "%s\n", "Send action goal!");
      sendActionGoal(bbox_detected);
    } else {
      if (!person_found) {
        person_saw_ = false;
        return;
      }
      RCLCPP_INFO(get_logger(), "%s\n", "Update goal!");
    }
    RCLCPP_INFO(get_logger(), "%s\n", "--------------------");
  }

private:
  void
  sendActionGoal(const gb_visual_detection_3d_msgs::msg::BoundingBox3d & bbox)
  {
    double orig_x, orig_y, orig_z;

    // Get the object psoe at its original frame:

    orig_x = (bbox.xmin + bbox.xmax) / 2.0;
    orig_y = (bbox.ymin + bbox.ymax) / 2.0;
    orig_z = 0.0;

    RCLCPP_INFO(get_logger(), "Coordinates=(%f, %f, %f)\n",
      orig_x, orig_y, orig_z);
  }

  bool
  newPerson(bool & found, gb_visual_detection_3d_msgs::msg::BoundingBox3d & out_bbox)
  {
    found = false;
    for (auto bbox : bboxes_) {
      if (bbox.object_name == "person") {
        found = true;
        out_bbox = bbox;
        if (!person_saw_) {
          return true;
        }
      }
    }
    return false;
  }

  void
  yolactCallback(const gb_visual_detection_3d_msgs::msg::BoundingBoxes3d::SharedPtr msg)
  {
    // Save the fields of the message

    bboxes_header_ = msg->header;
    bboxes_ = msg->bounding_boxes;
  }

  void
  initParams()
  {
    yolact_topic_ = "/yolact_ros2_3d/bounding_boxes_3d";
  }

  rclcpp::Subscription
  <gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>::SharedPtr yolact_sub_;

  std::string yolact_topic_;
  std_msgs::msg::Header bboxes_header_;
  std::vector<gb_visual_detection_3d_msgs::msg::BoundingBox3d> bboxes_;
  bool person_saw_;
};

int
main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Bboxes3d2nav2>("bboxes3d_to_nav2_node");

  rclcpp::Rate loop_rate(HZ);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node->get_node_base_interface());
    node->step();
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  exit(EXIT_SUCCESS);
}
