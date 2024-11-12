
#include "value_iteration2/Planner.h"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <algorithm>

namespace value_iteration2
{

vi_planner::vi_planner(const rclcpp::NodeOptions & options) : Node("vi_planner", options)
{
  RCLCPP_INFO(this->get_logger(), "vi_planner initialize start!");
  getParam();

  initPublisher();
  initSubscriber();
  initServiceServer();
  //initServiceClient();
  RCLCPP_INFO(this->get_logger(), "vi_planner initialize done!");
  //getCostMap2D();
}

void vi_planner::getParam()
{
  this->declare_parameter("use_dijkstra", false);
  this->declare_parameter("publish_searched_map", false);
  this->declare_parameter("update_path_weight", 0.05);
  this->declare_parameter("smooth_path_weight", 0.8);
  this->declare_parameter("iteration_delta_threshold", 1.e-6);

  this->get_parameter("use_dijkstra", use_dijkstra_);
  this->get_parameter("publish_searched_map", publish_searched_map_);
  this->get_parameter("update_path_weight", update_path_weight_);
  this->get_parameter("smooth_path_weight", smooth_path_weight_);
  this->get_parameter("iteration_delta_threshold", iteration_delta_threshold_);
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
    // RCLCPP_INFO(
    //   this->get_logger(), "Subscribed message at address: %p", static_cast<void *>(msg.get()));
    RCLCPP_INFO(this->get_logger(), "vi_planner get topic!");
    this->obstacle_map_ = *msg;
    initPlanner();
  };

  rclcpp::SubscriptionOptions options;
  options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  costmap_2d_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap_2d", 2, costmap_2d_callback, options);

  RCLCPP_INFO(this->get_logger(), "set subscriver!");
}

void vi_planner::initServiceServer()
{
  auto get_path = [&](
                    const std::shared_ptr<rmw_request_id_t> request_header,
                    const std::shared_ptr<value_iteration2_astar_msgs::srv::GetPath_Request> request,
                    std::shared_ptr<value_iteration2_astar_msgs::srv::GetPath_Response> response) -> void {
    (void)request_header;
    // clang-format off
    RCLCPP_INFO(this->get_logger(), "vi_planner planning start");
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
    RCLCPP_INFO(this->get_logger(), "origin x:%lf y:%lf",search_map_.info.origin.position.x,search_map_.info.origin.position.y);
    for(auto &i : response->path.poses){
      i.pose.position.x += obstacle_map_.info.origin.position.x;
      i.pose.position.y += obstacle_map_.info.origin.position.y;
      RCLCPP_INFO(this->get_logger(), "path x:%lf, y:%lf",i.pose.position.x, i.pose.position.y);
    }
    RCLCPP_INFO(this->get_logger(), "vi_planner planning done");
    initPlanner(); 
  };
  get_path_srv_ = create_service<value_iteration2_astar_msgs::srv::GetPath>("get_path", get_path);
  RCLCPP_INFO(this->get_logger(), "set service sever!");

}

/*void vi_planner::initServiceClient()
{
  get_costmap_2d_map_srv_client_ =
    this->create_client<value_iteration2_astar_msgs::srv::GetCostMap2D>("get_costmap_2d");
}*/

void vi_planner::initPlanner()
{
  RCLCPP_INFO(this->get_logger(), "vi_planner map setting start");
  resolution_ = obstacle_map_.info.resolution;
  robot_radius_ = 0.1;
  min_x_ = min_y_ = 0;
  max_x_ = x_width_ = obstacle_map_.info.width;
  max_y_ = y_width_ = obstacle_map_.info.height;
  motion_ = getMotionModel();
  search_map_ = obstacle_map_;
  RCLCPP_INFO(this->get_logger(), "vi_planner map setting done");
}

std::vector<std::tuple<int32_t, int32_t, uint8_t>> vi_planner::getMotionModel()
{
  // dx, dy, cost
  return std::vector<std::tuple<int32_t, int32_t, uint8_t>>{
    {1, 0, 1},
    {0, 1, 1},
    {-1, 0, 1},
    {0, -1, 1},
    {-1, -1, std::sqrt(2)},
    {-1, 1, std::sqrt(2)},
    {1, -1, std::sqrt(2)},
    {1, 1, std::sqrt(2)}};
}

nav_msgs::msg::Path vi_planner::planning(double sx, double sy, double gx, double gy)
{
  RCLCPP_INFO(this->get_logger(), "start x:%lf y:%lf, goal x:%lf y:%lf", sx,sy,gx,gy);
  RCLCPP_INFO(this->get_logger(), "hight:%lf width:%lf", resolution_*x_width_, resolution_*y_width_);
  auto start_node = value_iteration2::Node(calcXYIndex(sx), calcXYIndex(sy), 0.0, -1);
  auto goal_node = value_iteration2::Node(calcXYIndex(gx), calcXYIndex(gy), 0.0, -1);

  std::map<uint32_t, value_iteration2::Node> open_set, closed_set;
  open_set.insert(std::make_pair(calcGridIndex(start_node), start_node));

  //search_map_ = obstacle_map_;
  RCLCPP_INFO(this->get_logger(), "origin x:%lf y:%lf",search_map_.info.origin.position.x,search_map_.info.origin.position.y);
  RCLCPP_INFO(this->get_logger(), "start loop");
  
  while (rclcpp::ok()) {
    if (open_set.size() == 0) {
      RCLCPP_ERROR(this->get_logger(), "Open set is empty");
      rclcpp::shutdown();
    }

    // clang-format off
    auto c_id = [&]() -> uint32_t {
      std::map<uint32_t, double> id_cost_map;
      for (auto id_node_map : open_set) {
	      id_cost_map.insert(std::make_pair(
	        id_node_map.first, open_set[id_node_map.first].cost 
            + calcHeurisic(goal_node, static_cast<value_iteration2::Node>(open_set[id_node_map.first]))));
      }

      return std::min_element(
	       id_cost_map.begin(), id_cost_map.end(),
	       [](const auto & a, const auto & b) 
         { return a.second < b.second; }
         )->first;
    }();
    // clang-format on

    auto current = open_set[c_id];

    if (current.x == goal_node.x and current.y == goal_node.y) {
      RCLCPP_INFO(this->get_logger(), "Find goal");
      goal_node.parent_index = current.parent_index;
      goal_node.cost = current.cost;
      break;
    }

    open_set.erase(c_id);

    closed_set.insert(std::make_pair(c_id, current));

    for (size_t i = 0; i < motion_.size(); ++i) {
      auto node = value_iteration2::Node(
        current.x + std::get<0>(motion_[i]), current.y + std::get<1>(motion_[i]),
        current.cost + std::get<2>(motion_[i]), c_id);

      auto n_id = calcGridIndex(node);

      if (!verifyNode(node)) continue;
      // check motion
      search_map_.data[n_id] = 50;

      if (closed_set.find(n_id) != closed_set.end()) continue;
      if (open_set.find(n_id) == open_set.end())
        open_set.insert(std::make_pair(n_id, node));
      else {
        if (open_set[n_id].cost > node.cost) open_set.insert(std::make_pair(n_id, node));
      }
    }
  }

  return calcFinalPath(goal_node, closed_set);
}

nav_msgs::msg::Path vi_planner::calcFinalPath(
  value_iteration2::Node goal_node, std::map<uint32_t, value_iteration2::Node> closed_set)
{
  std::vector<double> rx, ry;
  rx.push_back(calcGridPosition(goal_node.x));
  ry.push_back(calcGridPosition(goal_node.y));

  auto parent_index = goal_node.parent_index;

  auto plan_path = nav_msgs::msg::Path();
  auto pose_stamp = geometry_msgs::msg::PoseStamped();

  while (parent_index != -1) {
    auto n = closed_set[parent_index];
    rx.push_back(calcGridPosition(n.x));
    ry.push_back(calcGridPosition(n.y));
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

  if (obstacle_map_.data[calcGridIndex(node)] == 100) return false;

  //一辺がrobot_radiusの正方形の範囲以内に使えないマスがあればfalse
  auto grid_robot_radius_ = std::ceil(robot_radius_ / obstacle_map_.info.resolution);
  for(auto i = node.y - grid_robot_radius_; i < node.y + grid_robot_radius_; i++)
    for(auto j = node.x - grid_robot_radius_; j < node.x + grid_robot_radius_; j++)
      if(obstacle_map_.data[i * x_width_ + j] > 10)
        return false;

  return true;
}

double vi_planner::calcHeurisic(value_iteration2::Node node1, value_iteration2::Node node2)
{
  auto w = 1.0;
  double d = w * std::hypot(
                   static_cast<double>(node1.x) - static_cast<double>(node2.x),
                   static_cast<double>(node1.y) - static_cast<double>(node2.y)) +
             obstacle_map_.data[calcGridIndex(node2)];

  // if Dijkstra's algorithm
  if (use_dijkstra_) d = 0.0;

  return d;
}

uint32_t vi_planner::calcXYIndex(double position)
{
  return static_cast<uint32_t>(std::round(position / resolution_));
}

uint32_t vi_planner::calcGridIndex(value_iteration2::Node node) { return node.y * x_width_ + node.x; }

/*
void vi_planner::getCostMap2D()
{
  while (!get_costmap_2d_map_srv_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
  }

  auto request = std::make_shared<ike_nav_msgs::srv::GetCostMap2D::Request>();
  using ServiceResponseFuture = rclcpp::Client<ike_nav_msgs::srv::GetCostMap2D>::SharedFuture;

  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto result = future.get();
    obstacle_map_ = result.get()->costmap_2d;
    initPlanner();
  };
  auto future_result =
    get_costmap_2d_map_srv_client_->async_send_request(request, response_received_callback);
}*/

}  // namespace value_iteration2

int main(int argc, char **argv)
{
	rclcpp::init(argc,argv);
  rclcpp::NodeOptions opt;
	auto node = std::make_shared<value_iteration2::vi_planner>(opt);
	rclcpp::spin(node);
	return 0;
}

