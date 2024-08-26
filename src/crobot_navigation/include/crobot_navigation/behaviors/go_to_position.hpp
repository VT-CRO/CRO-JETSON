#ifndef GO_TO_POSITION_HPP_
#define GO_TO_POSITION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

namespace BT
{
    template <> inline nav2_msgs::action::NavigateToPose_Goal convertFromString(StringView str)
    {
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input");
        } else {
            auto navGoal = NavigateToPose::Goal();
            navGoal.pose.header.frame_id = "map";
            navGoal.pose.pose.position.x = convertFromString<float>(parts[0]);
            navGoal.pose.pose.position.y = convertFromString<float>(parts[1]);

            tf2::Quaternion q;
            q.setRPY(0, 0, convertFromString<float>(parts[0]));
            navGoal.pose.pose.orientation = tf2::toMsg(q);

            return navGoal;
        }
    }
}


class GoToPosition : public BT::RosActionNode<NavigateToPose>
{
    public:
        GoToPosition(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);

        static BT::PortsList providedPorts()
        {
            // do stuff;
            return {
                BT::InputPort<nav2_msgs::action::NavigateToPose_Goal>("goal_pose")
            };
        }

        bool setGoal(RosActionNode<NavigateToPose>::Goal& goal) override;

        BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

        virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

        BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);
};

#endif