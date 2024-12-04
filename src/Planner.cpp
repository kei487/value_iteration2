
#include "value_iteration2/Planner.h"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <algorithm>

namespace value_iteration2
{

IkePlanner::IkePlanner(const rclcpp::NodeOptions & options) : Node("ike_planner", options)
{
  RCLCPP_INFO(this->get_logger(), "IkePlanner initialize start!");
  getParam();

  initPublisher();
  initSubscriber();
  initServiceServer();
  //initServiceClient();
  RCLCPP_INFO(this->get_logger(), "IkePlanner initialize done!");
  //getCostMap2D();
}

void IkePlanner::getParam()
{
<<<<<<< HEAD
<<<<<<< HEAD
  this->declare_parameter("use_dijkstra", false);
  this->declare_parameter("publish_searched_map", false);
  this->declare_parameter("update_path_weight", 0.05);
  this->declare_parameter("smooth_path_weight", 0.8);
  this->declare_parameter("iteration_delta_threshold", 1.e-6);
  this->declare_parameter("angle_resolution", 60);

  this->get_parameter("use_dijkstra", use_dijkstra_);
  this->get_parameter("publish_searched_map", publish_searched_map_);
  this->get_parameter("update_path_weight", update_path_weight_);
  this->get_parameter("smooth_path_weight", smooth_path_weight_);
  this->get_parameter("iteration_delta_threshold", iteration_delta_threshold_);
  this->get_parameter("angle_resolution", angle_resolution_);
=======
  this->param_listener_ =
    std::make_shared<ike_planner::ParamListener>(this->get_node_parameters_interface());

=======
  this->param_listener_ =
    std::make_shared<ike_planner::ParamListener>(this->get_node_parameters_interface());

>>>>>>> parent of 295dff8 (add param)
  use_dijkstra_ = this->params_.use_dijkstra;
  publish_searched_map_ = this->params_.publish_searched_map;

  update_path_weight_ = this->params_.update_path_weight;
  smooth_path_weight_ = this->params_.smooth_path_weight;
  iteration_delta_threshold_ = this->params_.iteration_delta_threshold;
<<<<<<< HEAD
>>>>>>> parent of 295dff8 (add param)
=======
>>>>>>> parent of 295dff8 (add param)
}

void IkePlanner::initPublisher()
{
  search_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "planner_searched_map", rclcpp::QoS(1).reliable());
  plan_path_pub_ =
    this->create_publisher<nav_msgs::msg::Path>("plan_path", rclcpp::QoS(1).reliable());
}

void IkePlanner::initSubscriber()
{
  auto costmap_2d_callback = [this](const nav_msgs::msg::OccupancyGrid::UniquePtr msg) {
    // RCLCPP_INFO(
    //   this->get_logger(), "Subscribed message at address: %p", static_cast<void *>(msg.get()));
    RCLCPP_INFO(this->get_logger(), "IkePlanner get topic!");
    this->obstacle_map_ = *msg;
    initPlanner();    
  };

  rclcpp::SubscriptionOptions options;
  options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  costmap_2d_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap_2d", 2, costmap_2d_callback, options);

  RCLCPP_INFO(this->get_logger(), "set subscriver!");
}

void IkePlanner::initServiceServer()
{
  auto get_path = [&](
                    const std::shared_ptr<rmw_request_id_t> request_header,
                    const std::shared_ptr<ike_nav_msgs::srv::GetPath_Request> request,
                    std::shared_ptr<ike_nav_msgs::srv::GetPath_Response> response) -> void {
    (void)request_header;
<<<<<<< HEAD
<<<<<<< HEAD
    RCLCPP_INFO(this->get_logger(), "vi_planner planning start");
=======
    // clang-format off
    RCLCPP_INFO(this->get_logger(), "IkePlanner planning start");
>>>>>>> parent of 295dff8 (add param)
=======
    // clang-format off
    RCLCPP_INFO(this->get_logger(), "IkePlanner planning start");
>>>>>>> parent of 295dff8 (add param)
    search_map_ = obstacle_map_;
    request->start.pose.position.x -= obstacle_map_.info.origin.position.x;
    request->start.pose.position.y -= obstacle_map_.info.origin.position.y;
    request->goal.pose.position.x -= obstacle_map_.info.origin.position.x;
    request->goal.pose.position.y -= obstacle_map_.info.origin.position.y;
    search_map_.info.origin.position.x -= obstacle_map_.info.origin.position.x;
    search_map_.info.origin.position.y -= obstacle_map_.info.origin.position.y;
    double start_angle,goal_angle;
    start_angle = 2*atan2(request->start.pose.orientation.z, request->start.pose.orientation.w) * 180 / M_PI;
    goal_angle = 2*atan2(request->goal.pose.orientation.z, request->goal.pose.orientation.w) * 180 / M_PI;
    RCLCPP_INFO(this->get_logger(), "g angle:%lf = atan2(%lf,%lf)", goal_angle, request->goal.pose.orientation.z, request->goal.pose.orientation.w);
    response->path = planning(
      request->start.pose.position.x, request->start.pose.position.y, start_angle, 
      request->goal.pose.position.x, request->goal.pose.position.y, goal_angle);
    RCLCPP_INFO(this->get_logger(), "origin x:%lf y:%lf",search_map_.info.origin.position.x,search_map_.info.origin.position.y);
<<<<<<< HEAD
    RCLCPP_INFO(this->get_logger(), "vi_planner planning done");
    //initPlanner(); 
=======
    for(auto &i : response->path.poses){
      i.pose.position.x += obstacle_map_.info.origin.position.x;
      i.pose.position.y += obstacle_map_.info.origin.position.y;
      RCLCPP_INFO(this->get_logger(), "path x:%lf, y:%lf",i.pose.position.x, i.pose.position.y);
    }
    RCLCPP_INFO(this->get_logger(), "IkePlanner planning done");
    initPlanner(); 
>>>>>>> parent of 295dff8 (add param)
  };
  get_path_srv_ = create_service<ike_nav_msgs::srv::GetPath>("get_path", get_path);
  RCLCPP_INFO(this->get_logger(), "set service sever!");

}

/*void IkePlanner::initServiceClient()
{
  get_costmap_2d_map_srv_client_ =
    this->create_client<ike_nav_msgs::srv::GetCostMap2D>("get_costmap_2d");
}*/

void IkePlanner::initPlanner()
{
  RCLCPP_INFO(this->get_logger(), "IkePlanner initializing start");
  resolution_ = obstacle_map_.info.resolution;
  robot_radius_ = 0.05;
  min_x_ = min_y_ = min_t_ = 0;
  max_x_ = x_width_ = obstacle_map_.info.width;
  max_y_ = y_width_ = obstacle_map_.info.height;
  max_t_ = std::floor(360 / angle_resolution_);
  motion_ = getMotionModel();
  search_map_ = obstacle_map_;
  RCLCPP_INFO(this->get_logger(), "IkePlanner initializing done");
}

std::vector<std::tuple<int32_t, int32_t, uint8_t>> IkePlanner::getMotionModel()
{
  // dx, dy, cost
  // liner, angler, cost
  return std::vector<std::tuple<int32_t, int32_t, uint8_t>>{
    {1, 0, 1},
    {-1, 0, 2},
    {0, -1, 1},
    {1, -1, 1},
    {0, 1, 1},
    {1, 1, 1}};
}

<<<<<<< HEAD
<<<<<<< HEAD
nav_msgs::msg::Path vi_planner::planning(double sx, double sy, double st, double gx, double gy, double gt)
=======
nav_msgs::msg::Path IkePlanner::planning(double sx, double sy, double gx, double gy)
>>>>>>> parent of 295dff8 (add param)
=======
nav_msgs::msg::Path IkePlanner::planning(double sx, double sy, double gx, double gy)
>>>>>>> parent of 295dff8 (add param)
{
  RCLCPP_INFO(this->get_logger(), "start x:%lf y:%lf t:%lf, goal x:%lf y:%lf t:%lf", sx,sy,st,gx,gy,gt);
  RCLCPP_INFO(this->get_logger(), "hight:%lf width:%lf", resolution_*x_width_, resolution_*y_width_);
  auto start_node = value_iteration2::Node(calcXYIndex(sx), calcXYIndex(sy), calcAIndex(st), 0.0, -1);
  auto goal_node = value_iteration2::Node(calcXYIndex(gx), calcXYIndex(gy), calcAIndex(gt), 0.0, -1);

  std::map<uint32_t, value_iteration2::Node> open_set, closed_set;
  open_set.insert(std::make_pair(calcNodeIndex(start_node), start_node));

  //search_map_ = obstacle_map_;
  RCLCPP_INFO(this->get_logger(), "origin x:%lf y:%lf",search_map_.info.origin.position.x,search_map_.info.origin.position.y);
  RCLCPP_INFO(this->get_logger(), "start loop");
  
  uint32_t cnt=0;
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

    if(cnt % 100 == 0){
      RCLCPP_INFO(this->get_logger(), "loop count:%d, open_set:%ld, heurisis:%lf", cnt, open_set.size(), calcHeurisic(goal_node,current));
      search_map_pub_->publish(search_map_);
    }
    cnt++;

    open_set.erase(c_id);

    closed_set.insert(std::make_pair(c_id, current));

    for (size_t i = 0; i < motion_.size(); ++i) {

			uint32_t dx, dy, dt;
			StateTransition(motion_[i], current.x, current.y, current.t, dx, dy, dt);
      auto node = value_iteration2::Node(dx, dy, dt, current.cost + std::get<2>(motion_[i]), c_id);
      auto n_id = calcNodeIndex(node);

      if (!verifyNode(node)) {
        continue;
      }
      // check motion
      search_map_.data[calcGridIndex(node)] = 100;

      if (closed_set.find(n_id) != closed_set.end()) {
        continue;
      }
      if (open_set.find(n_id) == open_set.end())
        open_set.insert(std::make_pair(n_id, node));
      else {
        if (open_set[n_id].cost > node.cost) open_set.insert(std::make_pair(n_id, node));
      }
    }
  }

  return calcFinalPath(goal_node, closed_set);
}

<<<<<<< HEAD
<<<<<<< HEAD
void vi_planner::StateTransition(std::tuple<int32_t, int32_t, uint8_t> motion, 
	uint32_t from_x, uint32_t from_y, uint32_t from_t, uint32_t &to_x, uint32_t &to_y, uint32_t &to_t)
{
  double dx, dy;
	double ang = calcAnglePosition(from_t) / 180 * M_PI;
	dx = calcGridPosition(from_x) + resolution_*std::get<0>(motion)*cos(ang);
	dy = calcGridPosition(from_y) + resolution_*std::get<0>(motion)*sin(ang);
  to_x = calcXYIndex(dx);
  to_y = calcXYIndex(dy); 

  if(static_cast<int32_t>(from_t) + std::get<1>(motion) < 0)
		to_t = from_t + std::get<1>(motion) + max_t_;
  else if(from_t + std::get<1>(motion) >= max_t_)
    to_t = from_t + std::get<1>(motion) - max_t_;
  else
    to_t = from_t + std::get<1>(motion);
  
}

nav_msgs::msg::Path vi_planner::calcFinalPath(
=======
nav_msgs::msg::Path IkePlanner::calcFinalPath(
>>>>>>> parent of 295dff8 (add param)
=======
nav_msgs::msg::Path IkePlanner::calcFinalPath(
>>>>>>> parent of 295dff8 (add param)
  value_iteration2::Node goal_node, std::map<uint32_t, value_iteration2::Node> closed_set)
{
  auto parent_index = goal_node.parent_index;

  auto plan_path = nav_msgs::msg::Path();
  auto pose_stamp = geometry_msgs::msg::PoseStamped();

  while (parent_index != -1) {
    auto n = closed_set[parent_index];
    pose_stamp.pose.position.x = calcGridPosition(n.x) + obstacle_map_.info.origin.position.x;
    pose_stamp.pose.position.y = calcGridPosition(n.y) + obstacle_map_.info.origin.position.y;
    pose_stamp.pose.orientation.w = cos( calcAnglePosition(n.t) / 2 );
    pose_stamp.pose.orientation.z = sin( calcAnglePosition(n.t) / 2 );
    RCLCPP_INFO(this->get_logger(), "path x:%lf, y:%lf, t:%lf",
      pose_stamp.pose.position.x, pose_stamp.pose.position.y, atan2(pose_stamp.pose.orientation.z,pose_stamp.pose.orientation.w) * 180 / M_PI );
    plan_path.poses.push_back(pose_stamp);
    parent_index = n.parent_index;
    search_map_.data[calcGridIndex(n)] = 50;
  }
  std::reverse(plan_path.poses.begin(), plan_path.poses.end());

  plan_path.header.frame_id = "map";
  plan_path.header.stamp = rclcpp::Time();

  if (publish_searched_map_) search_map_pub_->publish(search_map_);
  //smoothPath(plan_path);
  plan_path_pub_->publish(plan_path);

  return plan_path;
}

void IkePlanner::smoothPath(nav_msgs::msg::Path & path)
{
  auto smoothed_path = smoothOptimization(path);
  path = smoothed_path;
}

nav_msgs::msg::Path IkePlanner::smoothOptimization(nav_msgs::msg::Path & path)
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

double IkePlanner::calcNewPositionXY(
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

double IkePlanner::calcGridPosition(uint32_t node_position) { return node_position * resolution_; }
<<<<<<< HEAD

<<<<<<< HEAD
double vi_planner::calcAnglePosition(uint32_t node_position) { return node_position * angle_resolution_; }

bool vi_planner::verifyNode(value_iteration2::Node node)
=======
bool IkePlanner::verifyNode(value_iteration2::Node node)
>>>>>>> parent of 295dff8 (add param)
=======

bool IkePlanner::verifyNode(value_iteration2::Node node)
>>>>>>> parent of 295dff8 (add param)
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

  //半径robot_radius以内に使えないマスがあればfalse
  auto grid_robot_radius_ = std::ceil(robot_radius_ / obstacle_map_.info.resolution);
  for(auto i = node.y - grid_robot_radius_; i < node.y + grid_robot_radius_; i++)
    for(auto j = node.x - grid_robot_radius_; j < node.x + grid_robot_radius_; j++)
      if(obstacle_map_.data[i * x_width_ + j] > 10)
        return false;

  return true;
}

double IkePlanner::calcHeurisic(value_iteration2::Node node1, value_iteration2::Node node2)
{
  // node1 is goal , node2 is current
  // if Dijkstra's algorithm
  if (use_dijkstra_) return 0.0;

  auto w_d = 0.5;
  //auto w_ga = 0.1;
  auto w_gd = 1.0;
  auto current_t = calcAnglePosition(node2.t);
  auto goal_t = calcAnglePosition(node1.t);
  auto direction_t = atan2(node1.y-node2.y, node1.x-node2.x) * 180 / M_PI;
  auto diff_c2g = abs( current_t - goal_t );
  if (diff_c2g > 180) diff_c2g -= 180;
  auto diff_c2d = abs( current_t - direction_t );
  if (diff_c2d > 180) diff_c2d = 360 - diff_c2d;
  double value = w_d * std::hypot(
                   static_cast<double>(node1.x) - static_cast<double>(node2.x),
                   static_cast<double>(node1.y) - static_cast<double>(node2.y))
               // + w_ga * diff_c2g 
                + w_gd * diff_c2d; 
                 //obstacle_map_.data[calcGridIndex(node2)];

  return value;
}

uint32_t IkePlanner::calcXYIndex(double position)
{
  return static_cast<uint32_t>(std::round(position / resolution_));
}

<<<<<<< HEAD
<<<<<<< HEAD
uint32_t vi_planner::calcAIndex(double position)
{
  return static_cast<uint32_t>(std::round(position / angle_resolution_));
}

uint32_t vi_planner::calcGridIndex(value_iteration2::Node node) { return node.y * x_width_ + node.x ; }

uint32_t vi_planner::calcNodeIndex(value_iteration2::Node node) { return (node.y * x_width_ + node.x) * angle_resolution_ + node.t; }
=======
uint32_t IkePlanner::calcGridIndex(value_iteration2::Node node) { return node.y * x_width_ + node.x; }
>>>>>>> parent of 295dff8 (add param)

=======
uint32_t IkePlanner::calcGridIndex(value_iteration2::Node node) { return node.y * x_width_ + node.x; }

>>>>>>> parent of 295dff8 (add param)
void IkePlanner::getCostMap2D()
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
}

}  // namespace value_iteration2

int main(int argc, char **argv)
{
	rclcpp::init(argc,argv);
  rclcpp::NodeOptions opt;
	auto node = std::make_shared<value_iteration2::IkePlanner>(opt);
	rclcpp::spin(node);
	return 0;
}

