#ifndef PB2025_SENTRY_BEHAVIOR__NAVIGATION_GOAL_STATE_HPP_
#define PB2025_SENTRY_BEHAVIOR__NAVIGATION_GOAL_STATE_HPP_

#include <cmath>

#include "behaviortree_cpp/blackboard.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace pb2025_sentry_behavior
{
inline constexpr char kPublishedNavigationGoal[] = "_published_navigation_goal";

inline bool sameNavigationGoal(
  const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs)
{
  constexpr double tolerance = 1e-3;

  const auto & p1 = lhs.pose.position;

  const auto & p2 = rhs.pose.position;

  const auto & q1 = lhs.pose.orientation;

  const auto & q2 = rhs.pose.orientation;

  return lhs.header.frame_id == rhs.header.frame_id &&
         std::fabs(p1.x - p2.x) < tolerance && std::fabs(p1.y - p2.y) < tolerance &&
         std::fabs(p1.z - p2.z) < tolerance && std::fabs(q1.x - q2.x) < tolerance &&
         std::fabs(q1.y - q2.y) < tolerance && std::fabs(q1.z - q2.z) < tolerance &&
         std::fabs(q1.w - q2.w) < tolerance;
}

inline bool shouldPublishNavigationGoal(
  BT::Blackboard & blackboard, const geometry_msgs::msg::PoseStamped & goal,
  bool only_if_changed)
{
  auto * root = blackboard.rootBlackboard();

  geometry_msgs::msg::PoseStamped previous;
  
  const bool same = root->get(kPublishedNavigationGoal, previous) &&
                    sameNavigationGoal(previous, goal);
  if (only_if_changed && same) {
    return false;
  }
  root->set(kPublishedNavigationGoal, goal);
  return true;
}

inline void clearPublishedNavigationGoal(BT::Blackboard & blackboard)
{
  blackboard.rootBlackboard()->unset(kPublishedNavigationGoal);
}
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__NAVIGATION_GOAL_STATE_HPP_
