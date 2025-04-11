#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "patrolling_sim_ros2/srv/go_to_start_pos_srv.hpp"
#include "tf2/utils.h"
typedef rclcpp_action::Client<nav2_msgs::action::NavigateToPose> nav2_client;

using namespace std;

//ros::NodeHandle *n_ptr;
//rclcpp::Node::SharedPtr ptr;
std::shared_ptr<rclcpp::Node> n_ptr;
int teamsize;
double last_cmd_vel_time;

void cmd_velCB(const geometry_msgs::msg::Twist &msg)
{
    // ROS_INFO("receiving cmd_vels");
    last_cmd_vel_time = n_ptr->now().seconds();
}

bool GotoStartPosSrvCallback(const std::shared_ptr<patrolling_sim_ros2::srv::GoToStartPosSrv::Request> Req, std::shared_ptr<patrolling_sim_ros2::srv::GoToStartPosSrv::Response> Rep)
{

    if (Req->teamsize.data != teamsize)
    {
        RCLCPP_INFO(n_ptr->get_logger(), "Service was called with a different team size (%d) than expected (%d). Leaving.", Req->teamsize.data, teamsize);
        return false;
    }

    double starting_patrol_pos_x[teamsize];
    double starting_patrol_pos_y[teamsize];

    // list of doubles from the parameter server
    vector<double> initial_pos_list;
    auto parameter_ = rclcpp::Parameter("initial_pos",initial_pos_list);
    n_ptr->get_parameter("initial_pos",parameter_);
    //n_ptr->getParam("initial_pos", initial_pos_list);

    unsigned i;
    int j = 0; // robot id

    for (i = 0; i < initial_pos_list.size(); i++)
    {
        if (i % 2 == 0)
        { // even: x
            starting_patrol_pos_x[j] = initial_pos_list[i];
            // ROS_INFO("starting_patrol_pos_x[%d] = %f", j, starting_patrol_pos_x[j]);
        }
        else
        { // odd: y
            starting_patrol_pos_y[j] = initial_pos_list[i];
            // ROS_INFO("starting_patrol_pos_y[%d] = %f", j, starting_patrol_pos_y[j]);
            j++;
        }
    }

    // connect to move_base server and send robots one by one to starting positions...

    // array of pointers:
    //MoveBaseClient *ac_ptr[teamsize];
    std::shared_ptr<nav2_client> ac_ptr[teamsize];
    rclcpp::Rate loop_rate(1); // 1 sec

    for (j = teamsize - 1; j >= 0; j--)
    {

        char move_string[35];
        sprintf(move_string, "robot_%d/navigate_to_pose", j);
        // ROS_INFO("%s",move_string);

//        MoveBaseClient ac(move_string, true);
        std::shared_ptr<nav2_client> ac = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(n_ptr,move_string);
        ac_ptr[j] = ac;

        // wait for the action server to come up
        while (!ac->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_INFO(n_ptr->get_logger(), "Waiting for the nav2 action server to come up");
        }
        RCLCPP_INFO(n_ptr->get_logger(), "Connected with nav2 action server");

        //move_base_msgs::MoveBaseGoal goal;
        nav2_msgs::action::NavigateToPose::Goal goal;
        //geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(0.0);
        tf2::Quaternion tf2_quat;
        tf2_quat.setRPY(0.0,0.0,0.0);
        geometry_msgs::msg::Quaternion angle_quat = tf2::toMsg(tf2_quat);
        goal.pose.pose.orientation = angle_quat;

        // we'll send a goal to the robot to move 1 meter forward
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = n_ptr->now();
        goal.pose.pose.position.x = starting_patrol_pos_x[j];
        goal.pose.pose.position.y = starting_patrol_pos_y[j];
        //goal.target_pose.pose.orientation = angle_quat; // doesn't matter really.

        RCLCPP_INFO(n_ptr->get_logger(), "Sending goal");
        //ac.sendGoal(goal);
        ac->async_send_goal(goal);

        // wait a bit and send next goal to other robots.

        i = 0;

        while (i < Req->sleep_between_goals.data)
        {
            i++;
            //ros::spinOnce(); // trigger cmd_vel callbacks
            rclcpp::spin_some(n_ptr->get_node_base_interface()); // trigger cmd_vel callbacks
            loop_rate.sleep();
        }
    }

    RCLCPP_INFO(n_ptr->get_logger(), "Let's make sure that all robots reach their goals...!");

    last_cmd_vel_time = n_ptr->now().seconds();
    double current_time = n_ptr->now().seconds();

    while (current_time - last_cmd_vel_time < 5.0)
    { // check cmd_vels not received in the last 5 secs
        current_time = n_ptr->now().seconds();
        //ros::spinOnce();
        rclcpp::spin_some(n_ptr->get_node_base_interface());
        loop_rate.sleep();
    }

    RCLCPP_INFO(n_ptr->get_logger(), "All robots successfully reached their starting positions");
    return true;
}

int main(int argc, char **argv)
{

    //ros::init(argc, argv, "GoToStartPos");
    rclcpp::init(argc,argv);
    //ros::NodeHandle n;
    //n_ptr = &n;
    n_ptr = rclcpp::Node::make_shared("GoToStartPos");

    //ros::Duration(2.0).sleep(); // waiting for all navigation topics to show up.
    rclcpp::sleep_for(std::chrono::seconds(2));

    // get cmd_vel topics:
    //ros::master::V_TopicInfo master_topics;
    //ros::master::getTopics(master_topics);
    auto master_topics = n_ptr->get_topic_names_and_types();
    vector<string> cmd_vel_topic_array(32, "");
    teamsize = 0;

    for (const auto &n : master_topics){
        for (int i = 0; i<n.second.size(); i++)
        {
            if (n.second[0] == "geometry_msgs/msg/Twist")
            {
                cmd_vel_topic_array[teamsize] = n.first.c_str();
                teamsize++;
            }
        }
    }

    /*for (std::map<std::string,std::vector<std::string>>::iterator it 
            = master_topics.begin(); it != master_topics.end(); it++)
    {
        const std::map<std::string,std::vector<std::string>> info = it;
        if (info.datatype == "geometry_msgs/Twist")
        { // cmd_vel topics
            cmd_vel_topic_array[teamsize] = info.name;
            teamsize++;
        }
    }*/

    // for(int o=0; o<teamsize; o++){ ROS_ERROR("%s",cmd_vel_topic_array[o].c_str()); }

    if (teamsize == 0)
    {
        RCLCPP_ERROR(n_ptr->get_logger(), "No navigation information retrieved. Is \"nav2\" running?");
        return -1;
    }
    else
    {
        RCLCPP_INFO(n_ptr->get_logger(), "Detected %d robots.", teamsize);
    }

    //vector<ros::Subscriber> cmd_vel_sub(teamsize);
    vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> cmd_vel_sub(teamsize);

    for (int o = 0; o < teamsize; o++)
    { // create subscribers with a common callback:
        //cmd_vel_sub[o] = n.subscribe(cmd_vel_topic_array[o], 1, cmd_velCB);
        cmd_vel_sub[o] = n_ptr->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic_array[o], 1, cmd_velCB);
    }
    
    //ros::ServiceServer service = n.advertiseService("GotoStartPosSrv", GotoStartPosSrvCallback);
    rclcpp::Service<patrolling_sim_ros2::srv::GoToStartPosSrv>::SharedPtr service 
        = n_ptr->create_service<patrolling_sim_ros2::srv::GoToStartPosSrv>("go_to_start_pos_srv", &GotoStartPosSrvCallback);
    RCLCPP_WARN(n_ptr->get_logger(), "Ready to send robots to starting position.");

    //ros::spin();
    //rclcpp::spin();
    rclcpp::spin(n_ptr->get_node_base_interface());

    return 0;
}