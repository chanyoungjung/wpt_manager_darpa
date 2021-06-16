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
#include <ctime>
#include <fstream>
#include <mutex>
#include <queue>
#include <string>
#include <tuple>
#include <typeinfo>
#include <vector>

#include <kaist_drone_msgs/BehaviorNGoal.h>

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
std::vector<std::tuple<double, double, double>> wpt_xyz_global_coor;
std::vector<double> random_wpt_xyz;

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

std::tuple<double, double, double>
bodyCoordinateToGlobal(double body_x, double body_y, double body_z,
                       double altitude_limit = 2.5) {
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

  if (global_z > altitude_limit) {
    global_z = altitude_limit;
  }

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

std::tuple<double, double, double>
genRandomWPTwithInLocalCoor(double range_x_local_min, double range_x_local_max,
                            double range_y_local_min, double range_y_local_max,
                            double range_z_local_min,
                            double range_z_local_max) {

  double random_wpt_x =
      range_x_local_min +
      (rand() % static_cast<int>(range_x_local_max - range_x_local_min + 1));

  double random_wpt_y =
      range_y_local_min +
      (rand() % static_cast<int>(range_y_local_max - range_y_local_min + 1));

  double random_wpt_z =
      range_z_local_min +
      (rand() % static_cast<int>(range_z_local_max - range_z_local_min + 1));

  return bodyCoordinateToGlobal(random_wpt_x, random_wpt_y, random_wpt_z);
};

std::tuple<double, double, double> genRandomWPTwithInGlobalCoor(
    double range_x_global_min, double range_x_global_max,
    double range_y_global_min, double range_y_global_max,
    double range_z_global_min, double range_z_global_max) {

  double random_wpt_x =
      range_x_global_min +
      (rand() % static_cast<int>(range_x_global_max - range_x_global_min + 1));

  double random_wpt_y =
      range_y_global_min +
      (rand() % static_cast<int>(range_y_global_max - range_y_global_min + 1));

  double random_wpt_z =
      range_z_global_min +
      (rand() % static_cast<int>(range_z_global_max - range_z_global_min + 1));

  return std::make_tuple(random_wpt_x, random_wpt_y, random_wpt_z);
};

vector<vector<double>> parse2DCsvFile(string inputFileName) {

  vector<vector<double>> data;
  ifstream inputFile(inputFileName);
  int l = 0;

  while (inputFile) {
    l++;
    string s;
    if (!getline(inputFile, s))
      break;
    if (s[0] != '#') {
      istringstream ss(s);
      vector<double> record;

      while (ss) {
        string line;
        if (!getline(ss, line, ','))
          break;
        try {
          record.push_back(stof(line));
        } catch (const std::invalid_argument e) {
          cout << "NaN found in file " << inputFileName << " line " << l
               << endl;
          e.what();
        }
      }

      data.push_back(record);
    }
  }

  if (!inputFile.eof()) {
    cerr << "Could not read file " << inputFileName << "\n";
    __throw_invalid_argument("File not found.");
  }

  return data;
}

std::vector<std::tuple<double, double, double>>
parse3DCsvFile(string inputFileName) {

  std::vector<std::tuple<double, double, double>> data;
  ifstream inputFile(inputFileName);
  int l = 0;

  while (inputFile) {
    l++;
    string s;
    if (!getline(inputFile, s))
      break;
    if (s[0] != '#') {
      istringstream ss(s);
      std::tuple<double, double, double> record(0.0, 0.0, 0.0);

      int cnt = 0;
      bool nan_flg = false;
      while (ss) {
        string line;
        if (!getline(ss, line, ','))
          break;
        try {
          if (cnt == 0)
            std::get<0>(record) = stof(line);
          else if (cnt == 1)
            std::get<1>(record) = stof(line);
          else if (cnt == 2)
            std::get<2>(record) = stof(line);
        } catch (const std::invalid_argument e) {
          cout << "NaN found in file " << inputFileName << " line " << l
               << endl;
          e.what();
          nan_flg = true;
        }
        cnt++;
      }
      if (nan_flg == false)
        data.push_back(record);
    }
  }

  if (!inputFile.eof()) {
    cerr << "Could not read file " << inputFileName << "\n";
    __throw_invalid_argument("File not found.");
  }

  return data;
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "random_wpt_generator");
  ROS_INFO("Initiated random_wpt_generator_node");

  ros::NodeHandle nh("~");
  ros::Rate r(10);

  bool random_wpt_generation_mode;
  string wpt_file_path;
  string wpt_topic_name;
  string odom_topic_name;
  int max_num_wpt;
  int num_wpt;
  string random_generation_coordination;
  double altitude_limit;
  double goal_arrived_boundary;
  double random_generation_body_x_min, random_generation_body_x_max,
      random_generation_body_y_min, random_generation_body_y_max,
      random_generation_body_z_min, random_generation_body_z_max,
      random_generation_global_x_min, random_generation_global_x_max,
      random_generation_global_y_min, random_generation_global_y_max,
      random_generation_global_z_min, random_generation_global_z_max;

  nh.getParam("random_generation", random_wpt_generation_mode);
  nh.getParam("predefined_wpt_file_path", wpt_file_path);
  nh.getParam("pub_wpt_topic_name", wpt_topic_name);
  nh.getParam("sub_odom_topic_name", odom_topic_name);
  nh.getParam("max_num_of_wpt", max_num_wpt);
  nh.getParam("num_of_random_wpt", num_wpt);
  nh.getParam("predefined_altitude_limit", altitude_limit);
  nh.getParam("goal_arrived_dist", goal_arrived_boundary);
  nh.getParam("random_generation_coordination", random_generation_coordination);
  nh.getParam("random_generation_body_x_min", random_generation_body_x_min);
  nh.getParam("random_generation_body_x_max", random_generation_body_x_max);
  nh.getParam("random_generation_body_y_min", random_generation_body_y_min);
  nh.getParam("random_generation_body_y_max", random_generation_body_y_max);
  nh.getParam("random_generation_body_z_min", random_generation_body_z_min);
  nh.getParam("random_generation_body_z_max", random_generation_body_z_max);
  nh.getParam("random_generation_global_x_min", random_generation_global_x_min);
  nh.getParam("random_generation_global_x_max", random_generation_global_x_max);
  nh.getParam("random_generation_global_y_min", random_generation_global_y_min);
  nh.getParam("random_generation_global_y_max", random_generation_global_y_max);
  nh.getParam("random_generation_global_z_min", random_generation_global_z_min);
  nh.getParam("random_generation_global_z_max", random_generation_global_z_max);

  // ros::Publisher local_random_goalAction_publisher =
  // nh.advertise<std_msgs::Float32MultiArray>("/scout/GoalAction", 10);

  vector<vector<double>> wpt_xy;

  if (random_wpt_generation_mode == false) {
    // predifined wpt mode
    // read csv file
    wpt_xyz_global_coor = parse3DCsvFile(wpt_file_path);
    if (wpt_xyz_global_coor.size() == 0) {
      std::cout << "WPT FILE IS EMPTY" << std::endl;
      std::cout << "NODE END" << std::endl;
      return 0;
    } else {
      std::cout << "VERSION : PREDIFINED WPT" << std::endl;
      std::cout << "# OF WPT : " << wpt_xyz_global_coor.size() << std::endl;
      for (int wpt_idx = 0; wpt_idx < wpt_xyz_global_coor.size(); wpt_idx++) {
        std::cout << "WPT " << wpt_idx << ": "
                  << " X=" << std::get<0>(wpt_xyz_global_coor[wpt_idx])
                  << " Y=" << std::get<1>(wpt_xyz_global_coor[wpt_idx])
                  << " Z=" << std::get<2>(wpt_xyz_global_coor[wpt_idx])
                  << std::endl;
      }
    }
  } else {
    // random wpt mode
    if (random_generation_coordination == "inBodyCoordinate") {
      if (num_wpt > max_num_wpt)
        num_wpt = max_num_wpt;

      current_pose.position.x = 0.0;
      current_pose.position.y = 0.0;
      current_pose.position.z = 0.0;
      current_heading_yaw_inGlobalCoor = 0.0;
      for (int wpt_idx = 0; wpt_idx < num_wpt; wpt_idx++) {
        auto random_wpt_in_global_coor = genRandomWPTwithInLocalCoor(
            random_generation_body_x_min, random_generation_body_x_max,
            random_generation_body_y_min, random_generation_body_y_max,
            random_generation_body_z_min, random_generation_body_z_max);
        wpt_xyz_global_coor.push_back(random_wpt_in_global_coor);

        current_heading_yaw_inGlobalCoor += std::atan2(
            std::get<1>(random_wpt_in_global_coor) - current_pose.position.y,
            std::get<0>(random_wpt_in_global_coor) - current_pose.position.x);
        current_pose.position.x = std::get<0>(random_wpt_in_global_coor);
        current_pose.position.y = std::get<1>(random_wpt_in_global_coor);
        current_pose.position.z = std::get<2>(random_wpt_in_global_coor);
      }
      if (wpt_xyz_global_coor.size() == 0) {
        std::cout << "WPT RANDOM / GENERATION IN BODY COORDINATE" << std::endl;
        std::cout << "WPT EMPTY" << std::endl;
        std::cout << "NODE END" << std::endl;
        return 0;
      } else {
        std::cout << "VERSION : RANDOM WPT / GENERATION IN BODY COORDINATE"
                  << std::endl;
        std::cout << "# OF WPT : " << wpt_xyz_global_coor.size() << std::endl;
        for (int wpt_idx = 0; wpt_idx < wpt_xyz_global_coor.size(); wpt_idx++) {
          std::cout << "WPT " << wpt_idx << ": "
                    << " X=" << std::get<0>(wpt_xyz_global_coor[wpt_idx])
                    << " Y=" << std::get<1>(wpt_xyz_global_coor[wpt_idx])
                    << " Z=" << std::get<2>(wpt_xyz_global_coor[wpt_idx])
                    << std::endl;
        }
      }

    } else if (random_generation_coordination == "inGlobalCoordinate") {
      if (num_wpt > max_num_wpt)
        num_wpt = max_num_wpt;
      for (int wpt_idx = 0; wpt_idx < num_wpt; wpt_idx++) {
        auto random_wpt_in_global_coor = genRandomWPTwithInGlobalCoor(
            random_generation_global_x_min, random_generation_global_x_max,
            random_generation_global_y_min, random_generation_global_y_max,
            random_generation_global_z_min, random_generation_global_z_max);
        wpt_xyz_global_coor.push_back(random_wpt_in_global_coor);
      }
      if (wpt_xyz_global_coor.size() == 0) {
        std::cout << "WPT RANDOM / GENERATION IN GLOBAL COORDINATE"
                  << std::endl;
        std::cout << "WPT EMPTY" << std::endl;
        std::cout << "NODE END" << std::endl;
        return 0;
      } else {
        std::cout << "VERSION : RANDOM WPT / GENERATION IN GLOBAL COORDINATE"
                  << std::endl;
        std::cout << "# OF WPT : " << wpt_xyz_global_coor.size() << std::endl;
        for (int wpt_idx = 0; wpt_idx < wpt_xyz_global_coor.size(); wpt_idx++) {
          std::cout << "WPT " << wpt_idx << ": "
                    << " X=" << std::get<0>(wpt_xyz_global_coor[wpt_idx])
                    << " Y=" << std::get<1>(wpt_xyz_global_coor[wpt_idx])
                    << " Z=" << std::get<2>(wpt_xyz_global_coor[wpt_idx])
                    << std::endl;
        }
      }
    } else {
      wpt_xyz_global_coor.clear();
      std::cout << "RANDOM WPT GENERATION COORDINATION NAME IS WRONG"
                << std::endl;
      std::cout << "NODE END" << std::endl;
      return 0;
    }
  }

  //////////////////////////////
  // PUBLISHER AND SUBSCRIBER //
  //////////////////////////////
  ros::Subscriber cur_pose_subscriber =
      nh.subscribe(odom_topic_name, 10, current_pose_callback);
  ros::Publisher local_goal_publisher =
      nh.advertise<kaist_drone_msgs::BehaviorNGoal>(wpt_topic_name, 10);

  int current_wpt_idx = 0;
  while (ros::ok()) {

    if (current_wpt_idx == 0) {
      // /////////////
      // Take-off mode
      // /////////////
      double wpt_x = current_pose.position.x + 0.0;
      double wpt_y = current_pose.position.y + 0.0;
      double wpt_z = current_pose.position.z + 1.2;

      kaist_drone_msgs::BehaviorNGoal msg;
      geometry_msgs::PoseStamped pt_msg;
      pt_msg.header.stamp = ros::Time::now();
      pt_msg.header.frame_id = "odom";
      pt_msg.pose.position.x = wpt_x;
      pt_msg.pose.position.y = wpt_y;
      pt_msg.pose.position.z = wpt_z;
      msg.goal_pt_in_global = pt_msg;
      msg.mode = msg.TAKEOFF;
      local_goal_publisher.publish(msg);

      if (current_pose.position.z > wpt_z) {
        // /////////////
        // take-off done
        // /////////////
        current_wpt_idx++;
      }
    } else if (current_wpt_idx >= wpt_xyz_global_coor.size()) {
      // ////////////
      // Landing mode
      // ////////////
      double wpt_x = current_pose.position.x + 0.0;
      double wpt_y = current_pose.position.y + 0.0;
      double wpt_z = -100000000.0;

      kaist_drone_msgs::BehaviorNGoal msg;
      geometry_msgs::PoseStamped pt_msg;
      pt_msg.header.stamp = ros::Time::now();
      pt_msg.header.frame_id = "odom";
      pt_msg.pose.position.x = wpt_x;
      pt_msg.pose.position.y = wpt_y;
      pt_msg.pose.position.z = wpt_z;
      msg.goal_pt_in_global = pt_msg;
      msg.mode = msg.LAND;
      local_goal_publisher.publish(msg);
    } else {
      // /////////////
      // WPT following
      // /////////////
      double wpt_x = std::get<0>(wpt_xyz_global_coor[current_wpt_idx]);
      double wpt_y = std::get<1>(wpt_xyz_global_coor[current_wpt_idx]);
      double wpt_z = std::get<2>(wpt_xyz_global_coor[current_wpt_idx]);

      kaist_drone_msgs::BehaviorNGoal msg;
      geometry_msgs::PoseStamped pt_msg;
      pt_msg.header.stamp = ros::Time::now();
      pt_msg.header.frame_id = "odom";
      pt_msg.pose.position.x = wpt_x;
      pt_msg.pose.position.y = wpt_y;
      pt_msg.pose.position.z = wpt_z;
      msg.goal_pt_in_global = pt_msg;
      msg.mode = msg.WPT_FOLLOWING;
      local_goal_publisher.publish(msg);

      double dist_goal_to_robot = sqrt(pow(wpt_x - current_pose.position.x, 2) +
                                       pow(wpt_y - current_pose.position.y, 2));

      if (dist_goal_to_robot < goal_arrived_boundary) {
        // ///////////
        // WPT arrived
        // ///////////
        current_wpt_idx++;
      }
    }

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}