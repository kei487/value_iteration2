// SPDX-License-Identifier: Apache-2.0

#ifndef VALUE_ITERATION_PLANNER_H__K
#define VALUE_ITERATION_PLANNER_H__

// #include "ike_planner_parameter/ike_planner_parameter.hpp"

#include <rclcpp/rclcpp.hpp>

//#include "ike_nav_msgs/srv/get_cost_map2_d.hpp"
#include "value_iteration2_astar_msgs/srv/get_path.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

namespace value_iteration2
{
struct Node
{
  uint32_t x, y, t;
  double cost;
  int32_t parent_index;

  Node() : x(0), y(0), t(0), cost(0.0) {}
  Node(uint32_t x, uint32_t y, uint32_t t) : x(x), y(y), t(t), cost(0.0) {}
  Node(uint32_t x, uint32_t y, uint32_t t, double cost, int32_t parent_index)
  : x(x), y(y), t(t), cost(cost), parent_index(parent_index)
  {
  }
};

class IkePlanner : public rclcpp::Node
{
public:
  explicit IkePlanner(const rclcpp::NodeOptions & options);

protected:
  void getParam();

  void initPublisher();
  void initSubscriber();
  void initServiceServer();
  void initServiceClient();

  void initPlanner();

  void getCostMap2D();

  std::vector<std::tuple<int32_t, int32_t, uint8_t>> getMotionModel();

  nav_msgs::msg::Path planning(double sx, double sy, double st, double gx, double gy, double gt);
  uint32_t calcXYIndex(double positio);
  uint32_t calcAIndex(double positio);
  uint32_t calcGridIndex(value_iteration2::Node node);
  uint32_t calcNodeIndex(value_iteration2::Node node);
  double calcHeurisic(value_iteration2::Node node1, value_iteration2::Node node2);
  bool verifyNode(value_iteration2::Node node);
  nav_msgs::msg::Path calcFinalPath(
    value_iteration2::Node goal_node, std::map<uint32_t, value_iteration2::Node> closed_set);
  double calcGridPosition(uint32_t goal_node_position);
  double calcAnglePosition(uint32_t node_position);
  void StateTransition(std::tuple<int32_t, int32_t, uint8_t> motion, 
    uint32_t from_x, uint32_t from_y, uint32_t from_t, uint32_t &to_x, uint32_t &to_y, uint32_t &to_t);

  void smoothPath(nav_msgs::msg::Path & path);
  nav_msgs::msg::Path smoothOptimization(nav_msgs::msg::Path & path);
  double calcNewPositionXY(
    double & delta, double original_data, double smoothed_data, double smoothed_prev_data,
    double smoothed_next_data);

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr search_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_path_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_2d_sub_;
  rclcpp::Service<value_iteration2_astar_msgs::srv::GetPath>::SharedPtr get_path_srv_;
  //rclcpp::Client<ike_nav_msgs::srv::GetCostMap2D>::SharedPtr get_costmap_2d_map_srv_client_;

  //std::shared_ptr<ike_planner::ParamListener> param_listener_;
  ike_planner::Params params_;

  double resolution_, robot_radius_;
  uint32_t angle_resolution_;
  uint32_t min_x_, min_y_, min_t_, max_x_, max_y_, max_t_;
  nav_msgs::msg::OccupancyGrid obstacle_map_;
  nav_msgs::msg::OccupancyGrid search_map_;
  uint32_t x_width_, y_width_;
  std::vector<std::tuple<int32_t, int32_t, uint8_t>> motion_;

  bool use_dijkstra_, publish_searched_map_;
  double update_path_weight_, smooth_path_weight_, iteration_delta_threshold_;

  // todo
  double max_smooth_path_iteration_;
};

}  // namespace value_iteration2

#endif  // VALUE_ITERATION_PLANNER_H__