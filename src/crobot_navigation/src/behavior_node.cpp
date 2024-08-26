#include "crobot_navigation/behavior_node.hpp"
#include <behaviortree_ros2/bt_action_node.hpp>

#include "crobot_navigation/behaviors/go_to_position.hpp"

using namespace std::chrono_literals;

const std::string bt_xml_dir =
    ament_index_cpp::get_package_share_directory("crobot_navigation") + "/tree";

BehaviorNode::BehaviorNode(const std::string &node_name) 
    : Node(node_name)
{
    this->declare_parameter("location_file", "none");

    RCLCPP_INFO(get_logger(), "Init done");
}

void BehaviorNode::setup()
{
    create_behavior_tree();

    const auto timer_period = 500ms;
    timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&BehaviorNode::update_behavior_tree, this)
    );

    rclcpp::spin(shared_from_this());
    rclcpp::shutdown();
}

void BehaviorNode::create_behavior_tree()
{
    BT::BehaviorTreeFactory factory;

    auto node = std::make_shared<rclcpp::Node>("navigate_to_pose_action_client");
    BT::RosNodeParams params;
    params.nh = node;
    params.default_port_value = "navigate_to_pose";
    params.server_timeout = 3000ms;
    params.wait_for_server_timeout = 3000ms;

    factory.registerNodeType<GoToPosition>("GoToPosition", params);

    tree_ = factory.createTreeFromFile(bt_xml_dir + "/bt_default.xml");
}

void BehaviorNode::update_behavior_tree()
{
    BT::NodeStatus tree_status = tree_.tickExactlyOnce();

    if (tree_status == BT::NodeStatus::RUNNING)
    {
        return;
    } else if (tree_status == BT::NodeStatus::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Finished Navigation");
    } else if (tree_status == BT::NodeStatus::FAILURE)
    {
        RCLCPP_INFO(this->get_logger(), "Navigation Failed");
        // timer_->cancel();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BehaviorNode>("autonomy_node");
    node->setup();

    return 0;
}