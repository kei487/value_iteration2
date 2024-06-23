//SPDX-FileCopyrightText: 2024 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: BSD-3-Clause

#include "value_iteration2/vi_node.h"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <climits>
#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace value_iteration2{

ViNode::ViNode() : Node("vi_node")// : private_nh_("~"), yaw_(0.0), x_(0.0), y_(0.0), online_("false")
{
	setActions();
	declare_parameter("global_thread_num", 1);
	declare_parameter("cost_drawing_threshold", 60);

	int thread_num = get_parameter("global_thread_num").as_int();
	RCLCPP_INFO(this->get_logger(),"Global thread num: %d", thread_num);
	cost_drawing_threshold_ = get_parameter("cost_drawing_threshold").as_int();
	vi_.reset(new ValueIteratorLocal(*actions_, thread_num));
}

void ViNode::init(void)
{
	setCommunication();
        setMap();
	RCLCPP_INFO(get_logger(), "!!!!!!!!!!! INIT DONE !!!!!!!!!!ddd");
}

ViNode::~ViNode() 
{
	delete actions_;
}

void ViNode::setMap(void)
{
	declare_parameter("theta_cell_num", 60);
	declare_parameter("safety_radius", 0.2);
	declare_parameter("safety_radius_penalty", 30.0);
	declare_parameter("goal_margin_radius", 0.2);
	declare_parameter("goal_margin_theta", 10);
	declare_parameter("map_type", "occupancy");

	int theta_cell_num = get_parameter("theta_cell_num").as_int();
	double safety_radius = get_parameter("safety_radius").as_double();
	double safety_radius_penalty = get_parameter("safety_radius_penalty").as_double();
	double goal_margin_radius = get_parameter("goal_margin_radius").as_double();
	int goal_margin_theta = get_parameter("goal_margin_theta").as_int();

	std::string map_type = get_parameter("map_type").as_string();
	if(map_type == "occupancy"){
	
		while (true) {
			auto client = create_client<nav_msgs::srv::GetMap>("/map_server/map");
			while (!client->wait_for_service(1s)) {
				if (!rclcpp::ok()) {
					RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for map.");
					return;
				}
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "map server not available, waiting again...");
			}
		
			auto req = std::make_shared<nav_msgs::srv::GetMap::Request>();
			auto res = client->async_send_request(req);
			try{
			if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), res) 
					== rclcpp::FutureReturnCode::SUCCESS) {
				if (vi_->setMapWithOccupancyGrid(map_for_astar_ = res.get()->map,
					theta_cell_num, safety_radius, safety_radius_penalty,
					goal_margin_radius, goal_margin_theta)) {
					break;
				}
			} else {
				RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call map service");
			}
			}catch (const std::future_error& e) {
				RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Future error(setMap): %s", e.what());
			}
			sleep(1);
		}

	/*
	}else if(map_type == "cost"){
		while(!ros::service::waitForService("/cost_map", ros::Duration(3.0))){
			ROS_INFO("Waiting for cost_map");
		}

		ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetMap>("/cost_map");
		nav_msgs::GetMap::Request req;
		if(not client.call(req, res)){
			ROS_ERROR("cost_map not working");
			exit(1);
		}
		for(int i=0;i<100;i++)
			ROS_INFO("%u", (unsigned int)(res.map.data[i] & 0xFF));
	
		vi_->setMapWithCostGrid(res.map, theta_cell_num, safety_radius, safety_radius_penalty,
			goal_margin_radius, goal_margin_theta);
	*/
	}else{
		RCLCPP_INFO(this->get_logger(), "NO SUPPORT MAP TYPE");
		exit(1);
	}
}

void ViNode::setCommunication(void)
{
	declare_parameter("online", false);
	online_ = get_parameter("online").as_bool();
	RCLCPP_INFO(this->get_logger(),"Online: %s", online_ ? "true" : "false");

	if(online_){
		pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

		sub_laser_scan_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1,
				std::bind(&ViNode::scanReceived, this, std::placeholders::_1));
		RCLCPP_INFO(this->get_logger(),"set scan");
		sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 1,
				std::bind(&ViNode::goalReceived, this, std::placeholders::_1));

	        tf_listener_.reset();
	        tf_buffer_.reset();
	
	        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
	        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	}

	pub_value_function_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/value_function", 2);
	pub_cost_map_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap_2d", 2);

	decision_timer_ = this->create_wall_timer(100ms, std::bind(&ViNode::decision, this));
	value_pub_timer_ = this->create_wall_timer(1500ms, std::bind(&ViNode::pubValueFunction, this));

#if 0
	as_->start();
	srv_policy_ = nh_.advertiseService("/policy", &ViNode::servePolicy, this);
	srv_value_ = nh_.advertiseService("/value", &ViNode::serveValue, this);
#endif 
}

void ViNode::setActions(void)
{
	actions_ = new std::vector<Action>();
	actions_->push_back(Action("forward", 0.3, 0.0, 0));
	actions_->push_back(Action("back", -0.2, 0.0, 0));
	actions_->push_back(Action("right", 0.0, -20.0, 0));
	actions_->push_back(Action("rightfw", 0.2, -20.0, 0));
	actions_->push_back(Action("left", 0.0, 20.0, 0));
	actions_->push_back(Action("leftfw", 0.2, 20.0, 0));
}

void ViNode::scanReceived(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
	vi_->setLocalCost(msg, x_, y_, yaw_);
}

void ViNode::goalReceived(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) 
{
	vi_->idling_ = false;
	executeVi(msg);
}

/*
bool ViNode::servePolicy(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response)
{
	vi_->policyWriter(response);
	return true;
}

bool ViNode::serveValue(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response)
{
	vi_->valueFunctionWriter(response);
	return true;
}

*/

void ViNode::runThreads(void)
{
	vi_->setGoal(0.0, 0.0, 0);

	vector<thread> ths;
	for(int t=0; t<vi_->thread_num_; t++){
		ths.push_back(thread(&ValueIterator::valueIterationWorker, vi_.get(), INT_MAX, t));
		ths[t].detach();
	}

	if(online_)
		thread(&ValueIteratorLocal::localValueIterationWorker, vi_.get()).detach();
}

void ViNode::executeVi(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) 
{
	RCLCPP_INFO(get_logger(), "VALUE ITERATION START");
	auto &ori = msg->pose.orientation;	
	tf2::Quaternion q(ori.x, ori.y, ori.z, ori.w);
	double roll, pitch, yaw;
	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
	int t = (int)(yaw*180/M_PI);
	//RCLCPP_INFO(get_logger(), "GOAL: %lf %lf %d", msg->pose.position.x, msg->pose.position.y, t);
	vi_->setGoal(msg->pose.position.x, msg->pose.position.y, t);
	//RCLCPP_INFO(get_logger(), "START!!!");
	astar(msg);
	
	/*
	vector<thread> ths;
	for(int t=0; t<vi_->thread_num_; t++){
		ths.push_back(thread(&ValueIterator::valueIterationWorker, vi_.get(), INT_MAX, t));
		ths[t].detach();
	}

	if(online_)
		thread(&ValueIteratorLocal::localValueIterationWorker, vi_.get()).detach();
		*/
}

void ViNode::astar(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
	RCLCPP_INFO(get_logger(), "START A*!!!");

  //publish topic /costmap_2d nav_msgs::msg::OccupancyGrid
	pub_cost_map_->publish(map_for_astar_);

  //service client /get_path ike_nav_msgs::srv::GetPath
	get_path_srv_client_ =
    	this->create_client<ike_nav_msgs::srv::GetPath>("/get_path");

	//wait service avairable
	while(!get_path_srv_client_->wait_for_service(std::chrono::seconds(1))){
		if (!rclcpp::ok()) {
      		RCLCPP_ERROR(this->get_logger(),
				"client interrupted while waiting for service to appear.");
    	}
    	RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
	}

	//set request
	auto request = std::make_shared<ike_nav_msgs::srv::GetPath::Request>();

	request->goal.pose.position.x = msg->pose.position.x;
	request->goal.pose.position.y = msg->pose.position.y;

	try{
		tf_buffer_->canTransform("map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
		geometry_msgs::msg::TransformStamped trans =
					 tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
		request->start.pose.position.x  = trans.transform.translation.x;
		request->start.pose.position.y = trans.transform.translation.y;
	}catch(tf2::TransformException &e){
		RCLCPP_WARN(this->get_logger(),"current position error:%s", e.what());
	}

	//set response
	using ServiceResponseFuture = 
		rclcpp::Client<ike_nav_msgs::srv::GetPath>::SharedFuture;
	
	auto response_received_callback = [this](ServiceResponseFuture future) {
    	auto response = future.get();
    	//RCLCPP_INFO(this->get_logger(), "Path received");
	  	//make thread for VI
		std::reverse(response->path.poses.begin(), response->path.poses.end());
		int i=0;
		for(auto p : response->path.poses) {
			ths_a.push_back(thread(&ValueIterator::valueIterationWorkerAstar,vi_.get(),p,i++));
			//RCLCPP_INFO(get_logger(), 
			//	"path:x %lf,y %lf",i.pose.position.x,i.pose.position.y);
		}
		for(auto &t : ths_a) t.join();
	  	RCLCPP_INFO(get_logger(), "A* DONE!!!");
	};

	//call service
	auto future_result = 
		get_path_srv_client_->async_send_request(request, response_received_callback);
	

}

void ViNode::pubValueFunction(void)
{
	RCLCPP_INFO(get_logger(), "PUBLISH VALUE FUNC");
	nav_msgs::msg::OccupancyGrid map, local_map;

	vi_->makeValueFunctionMap(map, cost_drawing_threshold_, yaw_);
	pub_value_function_->publish(map);
}

void ViNode::decision(void)
{
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = 0.0;
	cmd_vel.angular.z = 0.0;

	if(not online_ or vi_->idling_) {
		pub_cmd_vel_->publish(cmd_vel);
		return; 
	}

	try{
		tf_buffer_->canTransform("map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
		geometry_msgs::msg::TransformStamped trans = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
		x_ = trans.transform.translation.x;
		y_ = trans.transform.translation.y;
		yaw_ = tf2::getYaw(trans.transform.rotation);
	}catch(tf2::TransformException &e){
		RCLCPP_WARN(this->get_logger(),"error in decision:%s", e.what());
	}

	RCLCPP_INFO(this->get_logger(),"X: %lf, Y: %lf, T: %lf", x_, y_, yaw_);
	vi_->setLocalWindow(x_, y_);

	Action *a = vi_->posToAction(x_, y_, yaw_);
	if(a != NULL){
		cmd_vel.linear.x = a->delta_fw_;
		cmd_vel.angular.z = a->delta_rot_/180*M_PI;
		pub_cmd_vel_->publish(cmd_vel);
	}
}

}

int main(int argc, char **argv)
{
	rclcpp::init(argc,argv);
	auto node = std::make_shared<value_iteration2::ViNode>();

	rclcpp::WallRate loop(10);
	node->init();
	node->runThreads();
	rclcpp::spin(node);
	return 0;
}
