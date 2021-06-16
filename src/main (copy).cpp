#include <cstdlib> // rand and srand
#include <ctime>
#include <float.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <cmath>
#include <fstream>
#include <mutex>
#include <queue>
#include <string>
#include <tuple>
#include <typeinfo>
#include <vector>

using namespace std;
#define DEG2RAD 0.0174533
#define FIXED_Z 1.5
#define RANDOM_MAX_COUNT 50

#define RANDOM_MAX_X 10.0
#define RANDOM_MIN_X 4.0
#define RANDOM_MAX_Y 10.0
#define RANDOM_MIN_Y -10.0

#define ARRIVAL_DIST 1.5

#define CONSERVATIVE_VEL 0.5
#define NORMAL_VEL 1.0
#define NORMAL_VEL_Z 0.5

#define AGGRESSIVE_VEL 1.5
#define TAKEOFF_VEL 0.5
#define LANDING_VEL 0.5

#define BATTERY_LOW_THRES 0.2

geometry_msgs::Pose current_pose;
double current_heading_yaw_inGlobalCoor;
int random_cnt = 0;
std::vector<double> random_wpt_xyz;
std::tuple<double, double, double> wpt_xyz_inGlobalCoor;
double cur_batt_level = 1; // 0~1

enum FLIGHT_SERVICE_TYPE {
  TAKE_OFF = 1,
  LANDING,
  WPT_FOLLOWING,
  PATH_FLIGHT,
  HOVERING
};

void current_pose_callback(const nav_msgs::Odometry::ConstPtr &message) {
  current_pose.position.x = message->pose.pose.position.x;
  current_pose.position.y = message->pose.pose.position.y;
  current_pose.position.z = message->pose.pose.position.z;
  current_pose.orientation.x = message->pose.pose.orientation.x;
  current_pose.orientation.y = message->pose.pose.orientation.y;
  current_pose.orientation.z = message->pose.pose.orientation.z;
  current_pose.orientation.w = message->pose.pose.orientation.w;

  if (current_pose.orientation.x == 0 && current_pose.orientation.y == 0 &&
      current_pose.orientation.z == 0 && current_pose.orientation.w == 0) {
    current_pose.orientation.x = 0;
    current_pose.orientation.y = 0;
    current_pose.orientation.z = 0;
    current_pose.orientation.w = 0.0001;
  }

  tf::Quaternion quat(current_pose.orientation.x, current_pose.orientation.y,
                      current_pose.orientation.z, current_pose.orientation.w);
  double global_roll, global_pitch, global_yaw;
  tf::Matrix3x3(quat).getRPY(global_roll, global_pitch, global_yaw);
  current_heading_yaw_inGlobalCoor = global_yaw;
}

void batt_level_callback(const sensor_msgs::BatteryState::ConstPtr &message) {
  cur_batt_level = message->percentage;
}

std::tuple<double, double, double>
bodyCoordinateToGlobal(double body_x, double body_y, double body_z) {
  double global_x, global_y, global_z;
  double body_vec_mag = sqrt(body_x * body_x + body_y * body_y);

  // global_x = current_pose.position.x +
  body_vec_mag *cos(current_heading_yaw_inGlobalCoor);
  global_x = current_pose.position.x +
             body_x * cos(current_heading_yaw_inGlobalCoor) -
             body_y * sin(current_heading_yaw_inGlobalCoor);
  //  global_y = current_pose.position.y +
  body_vec_mag *sin(current_heading_yaw_inGlobalCoor);
  global_y = current_pose.position.y +
             body_x * sin(current_heading_yaw_inGlobalCoor) +
             body_y * cos(current_heading_yaw_inGlobalCoor);
  global_z = current_pose.position.z + body_z;

  return make_tuple(global_x, global_y, global_z);
}

std::tuple<double, double, double>
globalCoordinateToBody(double target_global_x, double target_global_y,
                       double target_global_z) {
  double local_x = cos(-current_heading_yaw_inGlobalCoor) *
                       (target_global_x - current_pose.position.x) -
                   sin(-current_heading_yaw_inGlobalCoor) *
                       (target_global_y - current_pose.position.y);
  double local_y = sin(-current_heading_yaw_inGlobalCoor) *
                       (target_global_x - current_pose.position.x) +
                   cos(-current_heading_yaw_inGlobalCoor) *
                       (target_global_y - current_pose.position.y);
  double local_z = target_global_z - current_pose.position.z;

  return make_tuple(local_x, local_y, local_z);
}

void genRandomWPT(double range_x_min, double range_x_max, double range_y_min,
                  double range_y_max) {
  random_wpt_xyz.clear();

  unsigned seed = time(0);
  srand(seed);
  // double f = (double)rand() / RAND_MAX;
  // double ff = (double)rand() / RAND_MAX;
  double random_wpt_x_ =
      range_x_min + (rand() % static_cast<int>(range_x_max - range_x_min + 1));
  // double random_wpt_y_ = range_y_min + ff * (range_y_max - range_y_min);
  seed = time(0);
  srand(seed);
  double random_wpt_y_ =
      range_y_min + (rand() % static_cast<int>(range_y_max - range_y_min + 1));
  double random_wpt_z_ = FIXED_Z;

  random_wpt_xyz.push_back(random_wpt_x_);
  random_wpt_xyz.push_back(random_wpt_y_);
  random_wpt_xyz.push_back(random_wpt_z_);

  wpt_xyz_inGlobalCoor =
      bodyCoordinateToGlobal(random_wpt_x_, random_wpt_y_, random_wpt_z_);

  random_cnt++;
}; // in local coordinate

void readCSV(string csv_file_path) {
  ifstream fin;
  string line;
  // Open an existing file
  fin.open(csv_file_path);
  while (!fin.eof()) {
    fin >> line;
    cout << line << " ";
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "random_wpt_generator");
  ROS_INFO("Initiated random_wpt_generator_node");

  ros::NodeHandle nh("~");
  ros::Rate r(10);

  bool random_wpt_generation_mode;
  string wpt_file_path;
  string wpt_topic_name;
  int max_num_wpt;
  string random_generation_coordination;
  double random_generation_body_x_min, random_generation_body_x_max,
      random_generation_body_y_min, random_generation_body_y_max,
      random_generation_body_z_min, random_generation_body_z_max,
      random_generation_global_x_min, random_generation_global_x_max,
      random_generation_global_y_min, random_generation_global_y_max,
      random_generation_global_z_min, random_generation_global_z_max;

  nh.getParam("random_generation", random_wpt_generation_mode);
  nh.getParam("predefined_wpt_file_path", wpt_file_path);
  nh.getParam("pub_wpt_topic_name", wpt_topic_name);
  nh.getParam("max_num_of_wpt", max_num_wpt);
  nh.getParam("random_generation_coordination", random_generation_coordination);
  nh.getParam("random_generation_body_x_min", random_generation_body_x_min);
  nh.getParam("random_generation_body_x_max", random_generation_body_x_max);
  nh.getParam("random_generation_body_y_min", random_generation_body_y_min);
  nh.getParam("random_generation_body_y_max", random_generation_body_y_max);
  nh.getParam("random_generation_body_z_min", random_generation_body_z_min);
  nh.getParam("random_generation_body_z_max", random_generation_body_z_max);
  nh.getParam("random_generation_global_x_min", random_generation_global_x_min);
  nh.getParam("random_generation_global_x_min", random_generation_global_x_max);
  nh.getParam("random_generation_global_y_min", random_generation_global_y_min);
  nh.getParam("random_generation_global_y_min", random_generation_global_y_max);
  nh.getParam("random_generation_global_z_min", random_generation_global_z_min);
  nh.getParam("random_generation_global_z_min", random_generation_global_z_max);

  ros::Subscriber cur_pose_subscriber =
      nh.subscribe("/odom_ndt_global_prediction", 10, current_pose_callback);
  ros::Publisher local_random_goal_publisher =
      nh.advertise<nav_msgs::Path>(wpt_topic_name, 10);
  // ros::Publisher local_random_goalAction_publisher =
  // nh.advertise<std_msgs::Float32MultiArray>("/scout/GoalAction", 10);

  if (random_wpt_generation_mode == false) {
    // predifined wpt mode
    // read csv file

  } else {
    // random wpt mode
    if (random_generation_coordination == "bodyCoordinate") {

    } else if (random_generation_coordination == "globalCoordinate") {
    } else {
      std::cout << "RANDOM WPT GENERATION COORDINATION NAME IS WRONG"
                << std::endl;
      std::cout << "NODE END" << std::endl;
      return 0;
    }
  }

  while (ros::ok()) {

    if (random_cnt > RANDOM_MAX_COUNT || cur_batt_level < BATTERY_LOW_THRES) {

      /////////////////////////
      // SERVICE TYPE
      // GLOBAL GOAL X
      // GLOBAL GOAL X
      // GLOBAL GOAL Y
      // GLOBAL GOAL YAW
      // BODY X VELOCITY
      // BODY Y VELOCITY
      // BODY Z VELOCITY
      /////////////////////////
      std_msgs::Float32MultiArray ArrayMsg;
      int cur_flight_mode = LANDING;
      ArrayMsg.data.push_back(cur_flight_mode);         // service type
      ArrayMsg.data.push_back(current_pose.position.x); // global goal x
      ArrayMsg.data.push_back(current_pose.position.y); // global goal y
      ArrayMsg.data.push_back(
          -1000000.0); // global goal z --> around 1m height takeoff
      ArrayMsg.data.push_back(0.0);         // global goal yaw
      ArrayMsg.data.push_back(0.0);         // body x,y vel
      ArrayMsg.data.push_back(LANDING_VEL); // body z vel
      // local_random_goalAction_publisher.publish(ArrayMsg);

      nav_msgs::Path path_msg;
      geometry_msgs::PoseStamped pt_msg;
      pt_msg.header.frame_id = "baselink";
      pt_msg.pose.position.x = 0.0;
      pt_msg.pose.position.y = 0.0;
      pt_msg.pose.position.z = -1000000.0;
      path_msg.poses.push_back(pt_msg);
      local_random_goal_publisher.publish(path_msg);

    } else {

      if (random_cnt == 0) {
        genRandomWPT(RANDOM_MIN_X, RANDOM_MAX_X, RANDOM_MIN_Y, RANDOM_MAX_Y);
        /////////////////////////
        // SERVICE TYPE
        // GLOBAL GOAL X
        // GLOBAL GOAL X
        // GLOBAL GOAL Y
        // GLOBAL GOAL YAW
        // BODY X VELOCITY
        // BODY Y VELOCITY
        // BODY Z VELOCITY
        /////////////////////////
        std_msgs::Float32MultiArray ArrayMsg;
        int cur_flight_mode = TAKE_OFF;
        ArrayMsg.data.push_back(cur_flight_mode); // service type
        ArrayMsg.data.push_back(0.0);             // global goal x
        ArrayMsg.data.push_back(0.0);             // global goal y
        ArrayMsg.data.push_back(
            1.5); // global goal z --> around 1m height takeoff
        ArrayMsg.data.push_back(0.0);         // global goal yaw
        ArrayMsg.data.push_back(0.0);         // body x,y vel
        ArrayMsg.data.push_back(TAKEOFF_VEL); // body z vel
        // local_random_goalAction_publisher.publish(ArrayMsg);

        // global wpt to body coordinate conversion and publish
        auto local_goal = globalCoordinateToBody(0.0, 0.0, 2.5);
        nav_msgs::Path path_msg;
        geometry_msgs::PoseStamped pt_msg;
        pt_msg.header.frame_id = "baselink";
        pt_msg.pose.position.x = get<0>(local_goal);
        pt_msg.pose.position.y = get<1>(local_goal);
        pt_msg.pose.position.z = get<2>(local_goal);
        path_msg.poses.push_back(pt_msg);
        local_random_goal_publisher.publish(path_msg);

      } else {
        double distXY_to_wpt = sqrt(
            pow(get<0>(wpt_xyz_inGlobalCoor) - current_pose.position.x, 2) +
            pow(get<1>(wpt_xyz_inGlobalCoor) - current_pose.position.y, 2));

        if (distXY_to_wpt < ARRIVAL_DIST) {
          genRandomWPT(RANDOM_MIN_X, RANDOM_MAX_X, RANDOM_MIN_Y, RANDOM_MAX_Y);

          /////////////////////////
          // SERVICE TYPE
          // GLOBAL GOAL X
          // GLOBAL GOAL X
          // GLOBAL GOAL Y
          // GLOBAL GOAL YAW
          // BODY X VELOCITY
          // BODY Y VELOCITY
          // BODY Z VELOCITY
          /////////////////////////
          std_msgs::Float32MultiArray ArrayMsg;
          int cur_flight_mode = PATH_FLIGHT;
          ArrayMsg.data.push_back(cur_flight_mode); // service type
          ArrayMsg.data.push_back(
              get<0>(wpt_xyz_inGlobalCoor)); // global goal x
          ArrayMsg.data.push_back(
              get<1>(wpt_xyz_inGlobalCoor)); // global goal y
          ArrayMsg.data.push_back(
              get<2>(wpt_xyz_inGlobalCoor));     // global goal z
          ArrayMsg.data.push_back(0.0);          // global goal yaw
          ArrayMsg.data.push_back(NORMAL_VEL);   // body x,y vel
          ArrayMsg.data.push_back(NORMAL_VEL_Z); // body z vel
          // local_random_goalAction_publisher.publish(ArrayMsg);

          // global wpt to body coordinate conversion and publish
          auto local_goal = globalCoordinateToBody(
              get<0>(wpt_xyz_inGlobalCoor), get<1>(wpt_xyz_inGlobalCoor),
              get<2>(wpt_xyz_inGlobalCoor));
          nav_msgs::Path path_msg;
          geometry_msgs::PoseStamped pt_msg;
          pt_msg.header.frame_id = "baselink";
          pt_msg.pose.position.x = get<0>(local_goal);
          pt_msg.pose.position.y = get<1>(local_goal);
          pt_msg.pose.position.z = get<2>(local_goal);
          path_msg.poses.push_back(pt_msg);
          local_random_goal_publisher.publish(path_msg);
        } else {
          /////////////////////////
          // SERVICE TYPE
          // GLOBAL GOAL X
          // GLOBAL GOAL X
          // GLOBAL GOAL Y
          // GLOBAL GOAL YAW
          // BODY X VELOCITY
          // BODY Y VELOCITY
          // BODY Z VELOCITY
          /////////////////////////
          std_msgs::Float32MultiArray ArrayMsg;
          int cur_flight_mode = PATH_FLIGHT;
          ArrayMsg.data.push_back(cur_flight_mode); // service type
          ArrayMsg.data.push_back(
              get<0>(wpt_xyz_inGlobalCoor)); // global goal x
          ArrayMsg.data.push_back(
              get<1>(wpt_xyz_inGlobalCoor)); // global goal y
          ArrayMsg.data.push_back(
              get<2>(wpt_xyz_inGlobalCoor));     // global goal z
          ArrayMsg.data.push_back(0.0);          // global goal yaw
          ArrayMsg.data.push_back(NORMAL_VEL);   // body x,y vel
          ArrayMsg.data.push_back(NORMAL_VEL_Z); // body z vel
          // local_random_goalAction_publisher.publish(ArrayMsg);

          // global wpt to body coordinate conversion and publish
          auto local_goal = globalCoordinateToBody(
              get<0>(wpt_xyz_inGlobalCoor), get<1>(wpt_xyz_inGlobalCoor),
              get<2>(wpt_xyz_inGlobalCoor));
          nav_msgs::Path path_msg;
          geometry_msgs::PoseStamped pt_msg;
          pt_msg.header.frame_id = "baselink";
          pt_msg.pose.position.x = get<0>(local_goal);
          pt_msg.pose.position.y = get<1>(local_goal);
          pt_msg.pose.position.z = get<2>(local_goal);
          path_msg.poses.push_back(pt_msg);
          local_random_goal_publisher.publish(path_msg);
        }
      }
    }

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}