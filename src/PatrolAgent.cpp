/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, ISR University of Coimbra.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the ISR University of Coimbra nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: David Portugal (2011-2014), and Luca Iocchi (2014-2016)
 *********************************************************************/

#include "PatrolAgent.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <unistd.h>

using namespace std;
using namespace std::placeholders;

#define DELTA_TIME_SEQUENTIAL_START 15
#define SIMULATE_FOREVER                                                       \
  true // WARNING: Set this to false, if you want a finishing condition.

const std::string PS_path =
    ament_index_cpp::get_package_share_directory("patrolling_sim_ros2");

void PatrolAgent::init(int argc, char **argv) {
  /*
      argv[0]=/.../patrolling_sim/bin/GBS
      argv[1]=__name:=XXXXXX
      argv[2]=grid
      argv[3]=ID_ROBOT
  */
  this->n_ptr = std::make_shared<rclcpp::Node>("patrol_agent");
  this->exec.add_node(n_ptr);

  srand(time(NULL));

  // More than One robot (ID between 0 and 99)
  if (atoi(argv[3]) > NUM_MAX_ROBOTS || atoi(argv[3]) < -1) {
    RCLCPP_INFO(
        n_ptr->get_logger(),
        "The Robot's ID must be an integer number between 0 an 99"); // max 100
                                                                     // robots
    return;
  } else {
    ID_ROBOT = atoi(argv[3]);
    printf("ID_ROBOT = %d\n",
           ID_ROBOT); //-1 for 1 robot without prefix (robot_0)
  }

  /** D.Portugal: needed in case you "rosrun" from another folder **/
  chdir(PS_path.c_str());

  mapname = string(argv[2]);
  graph_file = "maps/" + mapname + "/" + mapname + ".graph";

  // Check Graph Dimension:
  dimension = GetGraphDimension(graph_file.c_str());

  // Create Structure to save the Graph Info;
  vertex_web = new vertex[dimension];

  // Get the Graph info from the Graph File
  GetGraphInfo(vertex_web, dimension, graph_file.c_str());

  uint nedges = GetNumberEdges(vertex_web, dimension);

  printf("Loaded graph %s with %d nodes and %d edges\n", mapname.c_str(),
         dimension, nedges);

#if 0
    /* Output Graph Data */   
    for (i=0;i<dimension;i++){
        printf ("ID= %u\n", vertex_web[i].id);
        printf ("X= %f, Y= %f\n", vertex_web[i].x, vertex_web[i].y);
        printf ("#Neigh= %u\n", vertex_web[i].num_neigh);
        
        for (j=0;j<vertex_web[i].num_neigh; j++){
        printf("\tID = %u, DIR = %s, COST = %u\n", vertex_web[i].id_neigh[j], vertex_web[i].dir[j], vertex_web[i].cost[j]);
        }
        
        printf("\n");   
    }
#endif

  interference = false;
  ResendGoal = false;
  goal_complete = true;
  last_interference = 0;
  goal_canceled_by_user = false;
  aborted_count = 0;
  resend_goal_count = 0;
  communication_delay = 0.0;
  lost_message_rate = 0.0;
  goal_reached_wait = 0.0;
  /* Define Starting Vertex/Position (Launch File Parameters) */

  // ros::init(argc, argv, "patrol_agent");  // will be replaced by
  // __name:=XXXXXX rclcpp::init(argc, argv);  // will be replaced by
  // __name:=XXXXXX ros::NodeHandle nh;

  // wait a random time (avoid conflicts with other robots starting at the same
  // time...)
  double r = 3.0 * ((rand() % 1000) / 1000.0);
  auto meanWaiting = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>{r});
  rclcpp::sleep_for(meanWaiting);

  double initial_x, initial_y;
  std::vector<double> list;

  n_ptr->declare_parameter<std::vector<double>>("initial_pos");
  rclcpp::Parameter list_param = n_ptr->get_parameter("initial_pos");
  list = list_param.as_double_array();

  if (list.empty()) {
    RCLCPP_ERROR(
        n_ptr->get_logger(),
        "No initial positions given: check \"initial_pos\" parameter.");
    rclcpp::shutdown();
    exit(-1);
  }

  int value = ID_ROBOT;
  if (value == -1) {
    value = 0;
  }

  initial_x = list[2 * value];
  initial_y = list[2 * value + 1];

  //   printf("initial position: x = %f, y = %f\n", initial_x, initial_y);
  current_vertex = IdentifyVertex(vertex_web, dimension, initial_x, initial_y);
  //   printf("initial vertex = %d\n\n",current_vertex);

  // instantaneous idleness and last visit initialized with zeros:
  instantaneous_idleness = new double[dimension];
  last_visit = new double[dimension];
  for (size_t i = 0; i < dimension; i++) {
    instantaneous_idleness[i] = 0.0;
    last_visit[i] = 0.0;

    if (i == current_vertex) {
      last_visit[i] = 0.1; // Avoids getting back at the initial vertex
    }
  }

  // Publicar dados de "odom" para nó de posições
  positions_pub =
      n_ptr->create_publisher<nav_msgs::msg::Odometry>("/positions", 1);

  // Subscrever posições de outros robots
  rclcpp::QoS policy = rclcpp::QoS(10);
  positions_sub = n_ptr->create_subscription<nav_msgs::msg::Odometry>(
      "/positions", policy,
      std::bind(&PatrolAgent::positionsCB, this, std::placeholders::_1));

  char string1[40];
  char string2[40];

  if (ID_ROBOT == -1) {
    strcpy(string1, "odom");    // string = "odom"
    strcpy(string2, "cmd_vel"); // string = "cmd_vel"
    TEAMSIZE = 1;
  } else {
    sprintf(string1, "/robot_%d/odom", ID_ROBOT);
    sprintf(string2, "/robot_%d/cmd_vel", ID_ROBOT);
    TEAMSIZE = ID_ROBOT + 1;
  }

  /* Set up listener for global coordinates of robots */
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(n_ptr->get_clock());
  listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Cmd_vel to backup:
  cmd_vel_pub = n_ptr->create_publisher<geometry_msgs::msg::Twist>(string2, 1);
  // Subscrever para obter dados de "odom" do robot corrente
  odom_sub = n_ptr->create_subscription<nav_msgs::msg::Odometry>(
      string1, 1, std::bind(&PatrolAgent::odomCB, this, _1));

  this->exec.spin_some();

  // Publicar dados para "results"
  results_pub =
      n_ptr->create_publisher<std_msgs::msg::Int16MultiArray>("/results", 100);
  results_sub = n_ptr->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/results", 100,
      std::bind(&PatrolAgent::resultsCB, this,
                _1)); // Subscrever "results" vindo dos robots

  // last time comm delay has been applied
  last_communication_delay_time = n_ptr->now().seconds();

  n_ptr->declare_parameter<double>("goal_reached_wait", 0.0);
  n_ptr->declare_parameter<double>("communication_delay", 0.0);
  n_ptr->declare_parameter<double>("lost_message_rate", 0.0);
  n_ptr->declare_parameter<string>("initial_positions", "default");

  readParams();
}

void PatrolAgent::ready() {

  char move_string[40];

  /* Define Goal */
  if (ID_ROBOT == -1) {
    strcpy(move_string, "navigate_to_pose"); // string = "move_base
  } else {
    sprintf(move_string, "/robot_%d/navigate_to_pose", ID_ROBOT);
  }

  this->ac = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      n_ptr, move_string);

  // wait for the action server to come up
  while (!this->ac->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_INFO(n_ptr->get_logger(),
                "Waiting for the nav2 action server to come up");
  }
  RCLCPP_INFO(n_ptr->get_logger(), "Connected with nav2 action server");

  initialize_node();         // announce that agent is alive
  rclcpp::Rate loop_rate(1); // 1 sec

  /* Wait until all nodes are ready.. */
  while (initialize) {
    this->exec.spin_once();
    loop_rate.sleep();
  }
}

void PatrolAgent::readParams() {

  if (!n_ptr->get_parameter("goal_reached_wait", goal_reached_wait)) {
    goal_reached_wait = 0.0;
    RCLCPP_WARN(
        n_ptr->get_logger(),
        "Cannot read parameter goal_reached_wait. Using default value!");
  } else {
    RCLCPP_INFO(n_ptr->get_logger(), "Parameter goal_reached_wait set to: %f",
                goal_reached_wait);
  }

  if (!n_ptr->get_parameter("communication_delay", communication_delay)) {
    communication_delay = 0.0;
    RCLCPP_WARN(
        n_ptr->get_logger(),
        "Cannot read parameter communication_delay. Using default value!");
  } else {
    RCLCPP_INFO(n_ptr->get_logger(), "Parameter communication_delay set to: %f",
                communication_delay);
  }

  if (!n_ptr->get_parameter("lost_message_rate", lost_message_rate)) {
    lost_message_rate = 0.0;
    RCLCPP_WARN(
        n_ptr->get_logger(),
        "Cannot read parameter lost_message_rate. Using default value!");
  } else {
    RCLCPP_INFO(n_ptr->get_logger(), "Parameter lost_message_rate set to: %f",
                lost_message_rate);
  }

  if (!n_ptr->get_parameter("initial_positions", initial_positions)) {
    initial_positions = "default";
    RCLCPP_WARN(
        n_ptr->get_logger(),
        "Cannot read parameter /initial_positions. Using default value '%s'!",
        initial_positions.c_str());
  } else {
    RCLCPP_INFO(n_ptr->get_logger(), "Parameter initial_positions set to %s",
                initial_positions.c_str());
  }
}

void PatrolAgent::run() {

  // get ready
  ready();

  // initially clear the costmap (to make sure the robot is not trapped):
  std::string mb_string;

  if (ID_ROBOT > -1) {
    std::ostringstream id_string;
    id_string << ID_ROBOT;
    mb_string = "/robot_" + id_string.str() + "/";
  }

  mb_string += "local_costmap/clear_entirely_local_costmap";
  this->clear_client = n_ptr->create_client<nav2_msgs::srv::ClearEntireCostmap>(
      mb_string.c_str());
  auto request =
      std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

  while (!this->clear_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(n_ptr->get_logger(), "Interrupted...");
      return;
    }
    RCLCPP_INFO(n_ptr->get_logger(), "Service not available, waiting...");
  }

  // auto result = this->clear_client->async_send_request(request);
  /*result.get();
  if (ros::service::call(mb_string.c_str(), srv)){
  //if (ros::service::call("move_base/clear_costmaps", srv)){
      RCLCPP_INFO(n_ptr->get_logger(),"Costmap correctly cleared before
  patrolling task."); }else{ RCLCPP_WARN(n_ptr->get_logger(),"Was not able to
  clear costmap (%s) before patrolling...", mb_string.c_str());
  }*/

  /* Run Algorithm */

  rclcpp::Rate loop_rate(30); // 0.033 seconds or 30Hz

  while (rclcpp::ok()) {

    if (goal_complete) {
      RCLCPP_INFO(n_ptr->get_logger(), "Goal Complete if case");
      onGoalComplete(); // can be redefined
      resend_goal_count = 0;
    } else { // goal not complete (active)
      if (interference) {
        RCLCPP_INFO(n_ptr->get_logger(), "Inference behaviour");
        do_interference_behavior();
      }

      if (ResendGoal) {
        // Send the goal to the robot (Global Map)
        if (resend_goal_count < 10) {
          resend_goal_count++;
          RCLCPP_INFO(n_ptr->get_logger(),
                      "Re-Sending goal (%d) - Vertex %d (%f,%f)",
                      resend_goal_count, next_vertex, vertex_web[next_vertex].x,
                      vertex_web[next_vertex].y);
          sendGoal(next_vertex);
        } else {
          resend_goal_count = 0;
          onGoalNotComplete();
        }
        ResendGoal = false; // para nao voltar a entrar (envia goal so uma vez)
      }

      processEvents();

      if (end_simulation) {
        return;
      }
    } // if (goal_complete)

    exec.spin_once();
    loop_rate.sleep();
  } // while ros.ok
}

void PatrolAgent::onGoalComplete() {
  if (next_vertex > -1) {
    // Update Idleness Table:
    update_idleness();
    current_vertex = next_vertex;
  }

  // devolver proximo vertex tendo em conta apenas as idlenesses;
  next_vertex = compute_next_vertex();
  // printf("Move Robot to Vertex %d (%f,%f)\n", next_vertex,
  // vertex_web[next_vertex].x, vertex_web[next_vertex].y);

  /** SEND GOAL (REACHED) AND INTENTION **/
  send_goal_reached(); // Send TARGET to monitor
  send_results();      // Algorithm specific function

  // Send the goal to the robot (Global Map)
  RCLCPP_INFO(n_ptr->get_logger(), "Sending goal - Vertex %d (%f,%f)\n",
              next_vertex, vertex_web[next_vertex].x,
              vertex_web[next_vertex].y);
  sendGoal(next_vertex); // send to move_base

  goal_complete = false;
}

void PatrolAgent::onGoalNotComplete() {
  int prev_vertex = next_vertex;

  RCLCPP_INFO(n_ptr->get_logger(),
              "Goal not complete - From vertex %d to vertex %d\n",
              current_vertex, next_vertex);

  // devolver proximo vertex tendo em conta apenas as idlenesses;
  next_vertex = compute_next_vertex();
  // printf("Move Robot to Vertex %d (%f,%f)\n", next_vertex,
  // vertex_web[next_vertex].x, vertex_web[next_vertex].y);

  // Look for a random adjacent vertex different from the previous one
  int random_cnt = 0;
  while (next_vertex == prev_vertex && random_cnt++ < 10) {
    int num_neighs = vertex_web[current_vertex].num_neigh;
    int i = rand() % num_neighs;
    next_vertex = vertex_web[current_vertex].id_neigh[i];
    RCLCPP_INFO(n_ptr->get_logger(), "Choosing another random vertex %d\n",
                next_vertex);
  }

  // Look for any random vertex different from the previous one
  while (next_vertex == prev_vertex && next_vertex == current_vertex) {
    int i = rand() % dimension;
    next_vertex = i;
    RCLCPP_INFO(n_ptr->get_logger(), "Choosing another random vertex %d\n",
                next_vertex);
  }

  // Send the goal to the robot (Global Map)
  RCLCPP_INFO(n_ptr->get_logger(), "Re-Sending NEW goal - Vertex %d (%f,%f)\n",
              next_vertex, vertex_web[next_vertex].x,
              vertex_web[next_vertex].y);
  sendGoal(next_vertex); // send to move_base

  goal_complete = false;
}

void PatrolAgent::processEvents() {}

void PatrolAgent::update_idleness() {
  double now = n_ptr->now().seconds();

  for (size_t i = 0; i < dimension; i++) {
    if ((int)i == next_vertex) {
      last_visit[i] = now;
    }
    instantaneous_idleness[i] = now - last_visit[i];

    // Show Idleness Table:
    // ROS_INFO("idleness[%u] = %f",i,instantaneous_idleness[i]);
  }
}

void PatrolAgent::initialize_node() { // ID,msg_type,1

  int value = ID_ROBOT;
  if (value == -1) {
    value = 0;
  }
  RCLCPP_INFO(n_ptr->get_logger(), "Initialize Node: Robot %d", value);

  std_msgs::msg::Int16MultiArray msg;
  msg.data.clear();
  msg.data.push_back(value);
  msg.data.push_back(INITIALIZE_MSG_TYPE);
  msg.data.push_back(1); // Robot initialized

  int count = 0;

  // ATENÇÃO ao PUBLICADOR!
  rclcpp::Rate loop_rate(0.5); // meio segundo

  while (count < 3) { // send activation msg 3times
    results_pub->publish(msg);
    this->exec.spin_some();
    loop_rate.sleep();
    count++;
  }
}

void PatrolAgent::getRobotPose(int robotid, float &x, float &y, float &theta) {

  if (listener == NULL) {
    RCLCPP_ERROR(n_ptr->get_logger(), "TF listener null");
    return;
  }

  std::stringstream ss;
  ss << "robot_" << robotid;
  std::string robotname = ss.str();
  std::string sframe = "map"; // Patch David Portugal: Remember that the global
                              // map frame is "/map"
  std::string dframe;

  if (ID_ROBOT > -1) {
    dframe = robotname + "/base_link";
  } else {
    dframe = "base_link";
  }

  geometry_msgs::msg::TransformStamped transform;
  tf2::Stamped<tf2::Transform> tf_stamped;

  try {
    transform = tf_buffer_->lookupTransform(sframe, dframe, rclcpp::Time(0),
                                            rclcpp::Duration::from_seconds(3));
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(n_ptr->get_logger(), "Cannot transform from %s to %s\n",
                 sframe.c_str(), dframe.c_str());
    RCLCPP_ERROR(n_ptr->get_logger(), "%s", ex.what());
  }

  x = transform.transform.translation.x;
  y = transform.transform.translation.y;
  theta = tf2::getYaw(transform.transform.rotation);
}

void PatrolAgent::odomCB(
    const nav_msgs::msg::Odometry &msg) { // colocar propria posicao na tabela
  int idx = ID_ROBOT;

  if (ID_ROBOT <= -1) {
    idx = 0;
  }

  float x, y, th;
  getRobotPose(idx, x, y, th);

  xPos[idx] = x; // msg->pose.pose.position.x;
  yPos[idx] = y; // msg->pose.pose.position.y;
}

void PatrolAgent::sendGoal(int next_vertex) {
  goal_canceled_by_user = false;

  double target_x = vertex_web[next_vertex].x,
         target_y = vertex_web[next_vertex].y;

  // Define Goal:
  nav2_msgs::action::NavigateToPose::Goal goal;
  // Send the goal to the robot (Global Map)

  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0.0, 0.0, 0.0);
  geometry_msgs::msg::Quaternion angle_quat = tf2::toMsg(tf2_quat);
  goal.pose.pose.orientation = angle_quat;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = n_ptr->now();
  goal.pose.pose.position.x = target_x;    // vertex_web[current_vertex].x;
  goal.pose.pose.position.y = target_y;    // vertex_web[current_vertex].y;
  goal.pose.pose.orientation = angle_quat; // doesn't matter really.

  auto send_goal_options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.feedback_callback =
      std::bind(&PatrolAgent::goalFeedbackCallback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&PatrolAgent::goalDoneCallback, this, _1);

  auto goal_handle_future = ac->async_send_goal(goal, send_goal_options);

  if (exec.spin_until_future_complete(goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(n_ptr->get_logger(), "send goal call failed");
    this->cancelGoal();
    ResendGoal = true;
  }

  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr
      goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(n_ptr->get_logger(), "Goal was rejected by the server");
    ResendGoal = true;
    goal_complete = false;
  } else {
    goal_complete = false;
    RCLCPP_INFO(n_ptr->get_logger(), "Goal was accepted by the server!");
  }
}

void PatrolAgent::cancelGoal() {
  goal_canceled_by_user = true;
  // ac->cancelAllGoals();
  auto cancel_future = ac->async_cancel_all_goals();

  if (exec.spin_until_future_complete(cancel_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(n_ptr->get_logger(), "Failed to cancel goal");
  } else {
    RCLCPP_INFO(n_ptr->get_logger(), "The goal was cancelled sucessfully!");
  }
}

void PatrolAgent::goalDoneCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::
        WrappedResult &result) { // goal terminado (completo ou cancelado)
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(n_ptr->get_logger(), "Goal reached ... WAITING %.2f sec",
                goal_reached_wait);

    auto goal_wait = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>{goal_reached_wait});
    rclcpp::sleep_for(goal_wait);
    RCLCPP_INFO(n_ptr->get_logger(), "Goal reached ... DONE");
    goal_complete = true;
  } else {
    aborted_count++;
    RCLCPP_INFO(n_ptr->get_logger(), "CANCELLED or ABORTED... %d",
                aborted_count); // tentar voltar a enviar goal..
    if (!goal_canceled_by_user) {
      RCLCPP_INFO(n_ptr->get_logger(),
                  "Goal not cancelled by the interference...");

      // RCLCPP_INFO(n_ptr->get_logger(),"Backup");
      backup();

      RCLCPP_INFO(n_ptr->get_logger(), "Resend Goal!");
      ResendGoal = true;
    }
  }
}

void PatrolAgent::goalActiveCallback(
    std::shared_future<rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::SharedPtr>
        future) { // enquanto o robot esta a andar para o goal...
  goal_complete = false;
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(n_ptr->get_logger(), "Goal was rejected by server!");
  } else {
    RCLCPP_INFO(n_ptr->get_logger(), "Goal was accepted by server!");
  }
}

void PatrolAgent::goalFeedbackCallback(
    rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>
        feedback) { // publicar posições

  // RCLCPP_INFO(n_ptr->get_logger(), "Goal feedback!");

  send_positions();

  int value = ID_ROBOT;
  if (value == -1) {
    value = 0;
  }
  interference = check_interference(value);
}

void PatrolAgent::send_goal_reached() {

  int value = ID_ROBOT;
  if (value == -1) {
    value = 0;
  }

  // [ID,msg_type,vertex,intention,0]
  std_msgs::msg::Int16MultiArray msg;
  msg.data.clear();
  msg.data.push_back(value);
  msg.data.push_back(TARGET_REACHED_MSG_TYPE);
  msg.data.push_back(current_vertex);

  results_pub->publish(msg);
  exec.spin_once(); // CHANGED THIS TO spin_some
}

bool PatrolAgent::check_interference(
    int robot_id) { // verificar se os robots estao proximos
  int i;
  double dist_quad;

  if (n_ptr->now().seconds() - last_interference < 10) // seconds
    return false; // false if within 7 seconds from the last one

  /* Poderei usar TEAMSIZE para afinar */
  for (i = 0; i < robot_id; i++) { // percorrer vizinhos (assim asseguro q cada
                                   // interferencia é so encontrada 1 vez)

    dist_quad = (xPos[i] - xPos[robot_id]) * (xPos[i] - xPos[robot_id]) +
                (yPos[i] - yPos[robot_id]) * (yPos[i] - yPos[robot_id]);

    if (dist_quad <=
        INTERFERENCE_DISTANCE *
            INTERFERENCE_DISTANCE) { // robots are ... meter or less apart
      last_interference = n_ptr->now().seconds();
      return true;
    }
  }
  return false;
}

void PatrolAgent::backup() {

  rclcpp::Rate loop_rate(100); // 100Hz

  int backUpCounter = 0;
  while (backUpCounter <= 100) {

    if (backUpCounter == 0) {
      RCLCPP_INFO(n_ptr->get_logger(),
                  "The wall is too close! I need to do some backing up...");
      // Move the robot back...
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = -0.1;
      cmd_vel.angular.z = 0.0;
      cmd_vel_pub->publish(cmd_vel);
    }

    if (backUpCounter == 20) {
      // Turn the robot around...
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.5;
      cmd_vel_pub->publish(cmd_vel);
    }

    if (backUpCounter == 100) {
      // Stop the robot...
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      cmd_vel_pub->publish(cmd_vel);

      RCLCPP_INFO(n_ptr->get_logger(), "Done backing up, now on with my life!");
    }

    // this->exec.spin_once();
    loop_rate.sleep();
    backUpCounter++;
  }
}

void PatrolAgent::do_interference_behavior() {
  RCLCPP_INFO(n_ptr->get_logger(),
              "Interference detected! Executing interference behavior...\n");
  send_interference(); // send interference to monitor for counting

#if 1
  // Stop the robot..
  cancelGoal();
  RCLCPP_INFO(n_ptr->get_logger(), "Robot stopped");
  // ros::Duration delay(3); // seconds
  // delay.sleep();
  rclcpp::sleep_for(std::chrono::seconds(7));
  ResendGoal = true;
#else
  // get own "odom" positions..
  ros::spinOnce();

  // Waiting until conflict is solved...
  int value = ID_ROBOT;
  if (value == -1) {
    value = 0;
  }
  while (interference) {
    interference = check_interference(value);
    if (goal_complete || ResendGoal) {
      interference = false;
    }
  }
#endif
}

// ROBOT-ROBOT COMMUNICATION
void PatrolAgent::send_positions() {
  // Publish Position to common node:
  nav_msgs::msg::Odometry msg;

  int idx = ID_ROBOT;

  if (ID_ROBOT <= -1) {
    msg.header.frame_id = "map"; // identificador do robot q publicou
    idx = 0;
  } else {
    char string[20];
    //sprintf(string, "robot_%d/map", ID_ROBOT);
    sprintf(string, "map");
    msg.header.frame_id = string;
  }

  msg.pose.pose.position.x = xPos[idx]; // send odometry.x
  msg.pose.pose.position.y = yPos[idx]; // send odometry.y
  // RCLCPP_INFO(n_ptr->get_logger(),"idx: %d x: %f y:
  // %f",idx,xPos[idx],yPos[idx]);

  positions_pub->publish(msg);
  // exec.spin_once();
  // this->exec.spin_once();
}

void PatrolAgent::receive_positions() {}

void PatrolAgent::positionsCB(
    const nav_msgs::msg::Odometry &msg) { // construir tabelas de posições

  // RCLCPP_INFO(n_ptr->get_logger(),"Construir tabela de posicoes (receber
  // posicoes), ID_ROBOT = %d\n",ID_ROBOT);

  char id[20]; // identificador do robot q enviou a msg d posição...
  strcpy(id, msg.header.frame_id.c_str());
  // int stamp = msg->header.seq;
  // RCLCPP_INFO(n_ptr->get_logger(),"robot q mandou msg = %s\n", id);

  // Build Positions Table

  if (ID_ROBOT > -1) {
    // verify id "XX" of robot: (string: "robotXXXX/map")

    char str_idx[4];
    uint i;

    for (i = 5; i < 9; i++) {
      if (id[i] == '/') {
        str_idx[i - 5] = '\0';
        break;
      } else {
        str_idx[i - 5] = id[i];
      }
    }

    int idx = atoi(str_idx);
    // RCLCPP_INFO(n_ptr->get_logger(),"id robot q mandou msg = %d\n",idx);

    if (idx >= TEAMSIZE && TEAMSIZE <= NUM_MAX_ROBOTS) {
      // update teamsize:
      TEAMSIZE = idx + 1;
    }

    if (ID_ROBOT != idx) { // Ignore own positions
      xPos[idx] = msg.pose.pose.position.x;
      yPos[idx] = msg.pose.pose.position.y;
    }
    // RCLCPP_INFO(n_ptr->get_logger(),"Position Table:\n frame.id = %s\n
    // id_robot = %d\n xPos[%d] = %f\n yPos[%d] = %f\n\n", id, idx, idx,
    // xPos[idx], idx, yPos[idx] );
  }

  receive_positions();
}

void PatrolAgent::send_results() {}

// simulates blocking send operation with delay in communication
void PatrolAgent::do_send_message(std_msgs::msg::Int16MultiArray &msg) {
  if (communication_delay > 0.001) {
    auto meanWaiting = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>{communication_delay});
    rclcpp::sleep_for(meanWaiting);
  }
  results_pub->publish(msg);
  exec.spin_once();
}

void PatrolAgent::receive_results() {}

void PatrolAgent::send_interference() {
  // interference: [ID,msg_type]

  int value = ID_ROBOT;
  if (value == -1) {
    value = 0;
  }
  printf("Send Interference: Robot %d\n", value);

  std_msgs::msg::Int16MultiArray msg;
  msg.data.clear();
  msg.data.push_back(value);
  msg.data.push_back(INTERFERENCE_MSG_TYPE);

  results_pub->publish(msg);
  this->exec.spin_once();
}

void PatrolAgent::resultsCB(const std_msgs::msg::Int16MultiArray &msg) {
  // RCLCPP_INFO(n_ptr->get_logger(), "Results CB");

  std::vector<signed short>::const_iterator it = msg.data.begin();

  vresults.clear();

  for (size_t k = 0; k < msg.data.size(); k++) {
    vresults.push_back(*it);
    it++;
  }

  int id_sender = vresults[0];
  int msg_type = vresults[1];

  // printf(" MESSAGE FROM %d TYPE %d ...\n",id_sender, msg_type);

  // messages coming from the monitor
  if (id_sender == -1 && msg_type == INITIALIZE_MSG_TYPE) {
    if (initialize == true &&
        vresults[2] == 100) { //"-1,msg_type,100,seq_flag" (BEGINNING)
      RCLCPP_INFO(n_ptr->get_logger(), "Let's Patrol!\n");

      double r = 1.0 * ((rand() % 1000) / 1000.0);
      auto meanWaiting = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>{r});

      rclcpp::sleep_for(meanWaiting);
      // TODO if sequential start
      // r = DELTA_TIME_SEQUENTIAL_START * ID_ROBOT;

      // ros::Duration wait(r); // seconds

      printf("Wait %.1f seconds (init pos:%s)\n", r, initial_positions.c_str());

      // wait.sleep();
      initialize = false;
    }

#if SIMULATE_FOREVER == false
    if (initialize == false && vresults[2] == 999) { //"-1,msg_type,999" (END)
      ROS_INFO("The simulation is over. Let's leave");
      end_simulation = true;
    }
#endif
  }

  if (!initialize) {
#if 0
        // communication delay
        if(ID_ROBOT>-1){
            if ((communication_delay>0.001) && (id_sender!=ID_ROBOT)) {
                    double current_time = ros::Time::now().toSec();
                    if (current_time-last_communication_delay_time>1.0) { 
                            ROS_INFO("Communication delay %.1f",communication_delay);
                            ros::Duration delay(communication_delay); // seconds
                            delay.sleep();
                            last_communication_delay_time = current_time;
                }
            }
            bool lost_message = false;
            if ((lost_message_rate>0.0001)&& (id_sender!=ID_ROBOT)) {
                double r = (rand() % 1000)/1000.0;
                lost_message = r < lost_message_rate;
            }
            if (lost_message) {
                ROS_INFO("Lost message");
            }
        }
#endif
    receive_results();
  }

  // exec.spin_once();
  // this->exec.spin_once();
}
