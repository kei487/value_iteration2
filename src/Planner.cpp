
#include "value_iteration2/Planner.h"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <algorithm>
#include <cmath>

namespace value_iteration2
{

vi_planner::vi_planner(const rclcpp::NodeOptions & options) : Node("vi_planner", options)
{
  RCLCPP_INFO(this->get_logger(), "vi_planner initialize start!");
  getParam();

  initPublisher();
  initSubscriber();
  initServiceServer();
  RCLCPP_INFO(this->get_logger(), "vi_planner initialize done!");
}

void vi_planner::getParam()
{
  this->declare_parameter("use_dijkstra", false);
  this->declare_parameter("publish_searched_map", false);
  this->declare_parameter("update_path_weight", 0.05);
  this->declare_parameter("smooth_path_weight", 0.8);
  this->declare_parameter("iteration_delta_threshold", 1.e-6);
  this->declare_parameter("lethal_cost_threshold", 100);
  this->declare_parameter("traversal_cost_weight", 1.0);
  this->declare_parameter("robot_radius", 0.1);

  this->get_parameter("use_dijkstra", use_dijkstra_);
  this->get_parameter("publish_searched_map", publish_searched_map_);
  this->get_parameter("update_path_weight", update_path_weight_);
  this->get_parameter("smooth_path_weight", smooth_path_weight_);
  this->get_parameter("iteration_delta_threshold", iteration_delta_threshold_);
  int lethal = 100;
  this->get_parameter("lethal_cost_threshold", lethal);
  lethal_cost_threshold_ = static_cast<unsigned int>(lethal);
  this->get_parameter("traversal_cost_weight", traversal_cost_weight_);
  this->get_parameter("robot_radius", robot_radius_);
}

void vi_planner::initPublisher()
{
  search_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "Planner_searched_map", rclcpp::QoS(1).reliable());
  plan_path_pub_ =
    this->create_publisher<nav_msgs::msg::Path>("plan_path", rclcpp::QoS(1).reliable());
}

void vi_planner::initSubscriber()
{
  auto costmap_2d_callback = [this](const nav_msgs::msg::OccupancyGrid::UniquePtr msg) {
    RCLCPP_INFO(this->get_logger(), "vi_planner get costmap (%ux%u)",
      msg->info.width, msg->info.height);
    this->obstacle_map_ = *msg;
    initPlanner();
  };

  /* /costmap_2d is latched (transient_local). Intra-process delivery rejects
   * any durability other than volatile, and this node runs in a separate
   * process from the publisher, so keep the subscription inter-process. */
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  costmap_2d_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap_2d", qos, costmap_2d_callback);

  RCLCPP_INFO(this->get_logger(), "set subscriber!");
}

void vi_planner::initServiceServer()
{
  auto get_path = [&](
                    const std::shared_ptr<rmw_request_id_t> request_header,
                    const std::shared_ptr<value_iteration2_astar_msgs::srv::GetPath_Request> request,
                    std::shared_ptr<value_iteration2_astar_msgs::srv::GetPath_Response> response) -> void {
    (void)request_header;
    RCLCPP_INFO(this->get_logger(), "vi_planner planning start");
    if (obstacle_map_.data.empty() || obstacle_map_.info.width == 0 || obstacle_map_.info.height == 0) {
      RCLCPP_ERROR(this->get_logger(), "costmap is empty; cannot plan");
      response->path = nav_msgs::msg::Path();
      return;
    }

    search_map_ = obstacle_map_;
    request->start.pose.position.x -= obstacle_map_.info.origin.position.x;
    request->start.pose.position.y -= obstacle_map_.info.origin.position.y;
    request->goal.pose.position.x -= obstacle_map_.info.origin.position.x;
    request->goal.pose.position.y -= obstacle_map_.info.origin.position.y;
    search_map_.info.origin.position.x -= obstacle_map_.info.origin.position.x;
    search_map_.info.origin.position.y -= obstacle_map_.info.origin.position.y;
    response->path = planning(
      request->start.pose.position.x, request->start.pose.position.y,
      request->goal.pose.position.x, request->goal.pose.position.y);
    for(auto &i : response->path.poses){
      i.pose.position.x += obstacle_map_.info.origin.position.x;
      i.pose.position.y += obstacle_map_.info.origin.position.y;
    }
    RCLCPP_INFO(this->get_logger(), "vi_planner planning done (%zu poses)",
      response->path.poses.size());
    initPlanner();
  };
  get_path_srv_ = create_service<value_iteration2_astar_msgs::srv::GetPath>("get_path", get_path);
  RCLCPP_INFO(this->get_logger(), "set service server!");
}

void vi_planner::initPlanner()
{
  RCLCPP_INFO(this->get_logger(), "vi_planner map setting start");
  resolution_ = obstacle_map_.info.resolution;
  min_x_ = min_y_ = 0;
  max_x_ = x_width_ = obstacle_map_.info.width;
  max_y_ = y_width_ = obstacle_map_.info.height;
  motion_ = getMotionModel();
  search_map_ = obstacle_map_;
  RCLCPP_INFO(this->get_logger(), "vi_planner map setting done");
}

std::vector<std::tuple<int32_t, int32_t, double>> vi_planner::getMotionModel()
{
  // dx, dy, cost
  return std::vector<std::tuple<int32_t, int32_t, double>>{
    {1, 0, 1.0},
    {0, 1, 1.0},
    {-1, 0, 1.0},
    {0, -1, 1.0},
    {-1, -1, std::sqrt(2.0)},
    {-1, 1, std::sqrt(2.0)},
    {1, -1, std::sqrt(2.0)},
    {1, 1, std::sqrt(2.0)}};
}

unsigned int vi_planner::getCellCost(uint32_t index) const
{
  if (index >= obstacle_map_.data.size())
    return lethal_cost_threshold_ + 1;
  return static_cast<unsigned int>(static_cast<unsigned char>(obstacle_map_.data[index]));
}

bool vi_planner::isLethal(unsigned int cost) const
{
  /* Match value iteration cost mode: 0-100 free (with cost), otherwise prohibited. */
  return cost > lethal_cost_threshold_;
}

nav_msgs::msg::Path vi_planner::planning(double sx, double sy, double gx, double gy)
{
  RCLCPP_INFO(this->get_logger(), "start x:%lf y:%lf, goal x:%lf y:%lf", sx,sy,gx,gy);
  auto start_node = value_iteration2::Node(calcXYIndex(sx), calcXYIndex(sy), 0.0, -1);
  auto goal_node = value_iteration2::Node(calcXYIndex(gx), calcXYIndex(gy), 0.0, -1);

  if (!verifyNode(start_node) || !verifyNode(goal_node)) {
    RCLCPP_ERROR(this->get_logger(), "start or goal is in lethal cost");
    return nav_msgs::msg::Path();
  }

  std::map<uint32_t, value_iteration2::Node> open_set, closed_set;
  open_set.insert(std::make_pair(calcGridIndex(start_node), start_node));

  while (rclcpp::ok()) {
    if (open_set.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Open set is empty; no path found");
      return nav_msgs::msg::Path();
    }

    auto c_id = [&]() -> uint32_t {
      std::map<uint32_t, double> id_cost_map;
      for (const auto & id_node_map : open_set) {
        id_cost_map.insert(std::make_pair(
          id_node_map.first, id_node_map.second.cost
            + calcHeurisic(goal_node, id_node_map.second)));
      }

      return std::min_element(
         id_cost_map.begin(), id_cost_map.end(),
         [](const auto & a, const auto & b)
         { return a.second < b.second; }
         )->first;
    }();

    auto current = open_set[c_id];

    if (current.x == goal_node.x and current.y == goal_node.y) {
      RCLCPP_INFO(this->get_logger(), "Find goal");
      goal_node.parent_index = current.parent_index;
      goal_node.cost = current.cost;
      break;
    }

    open_set.erase(c_id);
    closed_set[c_id] = current;

    for (size_t i = 0; i < motion_.size(); ++i) {
      auto n_x = static_cast<int32_t>(current.x) + std::get<0>(motion_[i]);
      auto n_y = static_cast<int32_t>(current.y) + std::get<1>(motion_[i]);
      if (n_x < 0 || n_y < 0)
        continue;

      auto node = value_iteration2::Node(
        static_cast<uint32_t>(n_x), static_cast<uint32_t>(n_y),
        current.cost + std::get<2>(motion_[i]), c_id);

      auto n_id = calcGridIndex(node);
      if (!verifyNode(node)) continue;

      /* Prefer lower-cost cells on the costmap. */
      unsigned int cell_cost = getCellCost(n_id);
      node.cost += traversal_cost_weight_ * static_cast<double>(cell_cost);

      if (n_id < search_map_.data.size())
        search_map_.data[n_id] = 50;

      if (closed_set.find(n_id) != closed_set.end()) continue;
      auto open_it = open_set.find(n_id);
      if (open_it == open_set.end())
        open_set[n_id] = node;
      else if (open_it->second.cost > node.cost)
        open_it->second = node;
    }
  }

  return calcFinalPath(goal_node, closed_set);
}

nav_msgs::msg::Path vi_planner::calcFinalPath(
  value_iteration2::Node goal_node, std::map<uint32_t, value_iteration2::Node> closed_set)
{
  auto plan_path = nav_msgs::msg::Path();
  auto pose_stamp = geometry_msgs::msg::PoseStamped();

  pose_stamp.pose.position.x = calcGridPosition(goal_node.x);
  pose_stamp.pose.position.y = calcGridPosition(goal_node.y);
  plan_path.poses.push_back(pose_stamp);

  auto parent_index = goal_node.parent_index;
  while (parent_index != -1) {
    auto n = closed_set[parent_index];
    parent_index = n.parent_index;

    pose_stamp.pose.position.x = calcGridPosition(n.x);
    pose_stamp.pose.position.y = calcGridPosition(n.y);
    plan_path.poses.push_back(pose_stamp);
  }
  std::reverse(plan_path.poses.begin(), plan_path.poses.end());

  plan_path.header.frame_id = "map";
  plan_path.header.stamp = rclcpp::Time();

  if (publish_searched_map_) search_map_pub_->publish(search_map_);
  smoothPath(plan_path);
  plan_path_pub_->publish(plan_path);

  return plan_path;
}

void vi_planner::smoothPath(nav_msgs::msg::Path & path)
{
  if (path.poses.size() < 3)
    return;
  auto smoothed_path = smoothOptimization(path);
  path = smoothed_path;
}

nav_msgs::msg::Path vi_planner::smoothOptimization(nav_msgs::msg::Path & path)
{
  auto new_path = path;
  auto delta = iteration_delta_threshold_;

  while (delta >= iteration_delta_threshold_) {
    delta = 0.;
    for (size_t i = 1; i < new_path.poses.size() - 1; ++i) {
      new_path.poses[i].pose.position.x = calcNewPositionXY(
        delta, path.poses[i].pose.position.x, new_path.poses[i].pose.position.x,
        new_path.poses[i - 1].pose.position.x, new_path.poses[i + 1].pose.position.x);

      new_path.poses[i].pose.position.y = calcNewPositionXY(
        delta, path.poses[i].pose.position.y, new_path.poses[i].pose.position.y,
        new_path.poses[i - 1].pose.position.y, new_path.poses[i + 1].pose.position.y);
    }
  }

  return new_path;
}

double vi_planner::calcNewPositionXY(
  double & delta, double original_data, double smoothed_data, double smoothed_prev_data,
  double smoothed_next_data)
{
  auto before_smoothed_data = smoothed_data;

  smoothed_data +=
    update_path_weight_ * (original_data - smoothed_data) +
    smooth_path_weight_ * (smoothed_next_data + smoothed_prev_data - (2. * smoothed_data));
  delta += abs(smoothed_data - before_smoothed_data);

  return smoothed_data;
}

double vi_planner::calcGridPosition(uint32_t node_position) { return node_position * resolution_; }

bool vi_planner::verifyNode(value_iteration2::Node node)
{
  if (node.x < min_x_)
    return false;
  else if (node.y < min_y_)
    return false;
  else if (node.x >= max_x_)
    return false;
  else if (node.y >= max_y_)
    return false;

  if (isLethal(getCellCost(calcGridIndex(node))))
    return false;

  /* Robot footprint: only lethal cells block (soft costs remain traversable). */
  auto grid_robot_radius = static_cast<int>(std::ceil(robot_radius_ / resolution_));
  for (int i = static_cast<int>(node.y) - grid_robot_radius;
       i <= static_cast<int>(node.y) + grid_robot_radius; ++i) {
    for (int j = static_cast<int>(node.x) - grid_robot_radius;
         j <= static_cast<int>(node.x) + grid_robot_radius; ++j) {
      if (j < static_cast<int>(min_x_) || j >= static_cast<int>(max_x_) ||
          i < static_cast<int>(min_y_) || i >= static_cast<int>(max_y_))
        continue;
      uint32_t idx = static_cast<uint32_t>(i) * x_width_ + static_cast<uint32_t>(j);
      if (isLethal(getCellCost(idx)))
        return false;
    }
  }

  return true;
}

double vi_planner::calcHeurisic(value_iteration2::Node node1, value_iteration2::Node node2)
{
  if (use_dijkstra_)
    return 0.0;

  auto w = 1.0;
  return w * std::hypot(
                   static_cast<double>(node1.x) - static_cast<double>(node2.x),
                   static_cast<double>(node1.y) - static_cast<double>(node2.y));
}

uint32_t vi_planner::calcXYIndex(double position)
{
  return static_cast<uint32_t>(std::round(position / resolution_));
}

uint32_t vi_planner::calcGridIndex(value_iteration2::Node node) { return node.y * x_width_ + node.x; }

}  // namespace value_iteration2

int main(int argc, char **argv)
{
	rclcpp::init(argc,argv);
  rclcpp::NodeOptions opt;
	auto node = std::make_shared<value_iteration2::vi_planner>(opt);
	rclcpp::spin(node);
	return 0;
}
