//
// Created by mizu on 2026/4/16.
//

#include "Decider.h"

void Decider::AimAndFire(const std_msgs::msg::Float64::SharedPtr msg) {
	double angle = msg->data;
    fort_.TurnTo(angle);
    fire_timer_ = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&Decider::Fire, this));
}



#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Decider)