#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <cmath>
#include <cstdlib> // rand and srand
#include <ctime>
#include <float.h>
#include <fstream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <queue>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <string>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tuple>
#include <typeinfo>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <kaist_drone_msgs/BehaviorNGoal.h>
#include <kaist_drone_msgs/BehaviorNGoalArray.h>

#include <low_pass_filter.h>

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
// #define Height_Compensate_Factor=3;
#define BATTERY_LOW_THRES 0.2
#define TAKE_OFF_THRES 0.3
#define COMEBACK_MAX_TRAVEL_RANGE 700.0

kaist_drone_msgs::BehaviorNGoal msg_ref;

nav_msgs::Path astarpath;
geometry_msgs::Pose current_pose;
geometry_msgs::Pose past_pose;
bool pose_callback_first_run = true;
bool comeback_triggered = false;

geometry_msgs::PoseStamped rviz_goal_pose;
std::vector<geometry_msgs::PoseStamped> vec_rviz_goals;
kaist_drone_msgs::BehaviorNGoalArray rviz_goal_list;
kaist_drone_msgs::BehaviorNGoalArray backtracking_goal_list;
double current_heading_yaw_inGlobalCoor;
bool is_land = false;
int random_cnt = 0;
std::vector<std::tuple<double, double, double>> wpt_xyz_global_coor;
std::vector<double> random_wpt_xyz;
bool is_hover = false;
bool is_skip = false;
double cur_batt_level = 1; // 0~1
int current_wpt_idx = 0;
double dist_next_goal_to_robot;
double max_z;
double min_z;
double max_y;
double min_y;
double max_x;
double min_x;
double look_ahead_dist = 2.0;
double astar_resolution = 0.1;
int look_ahead_ind = look_ahead_dist / astar_resolution;
double sub_goal_local_x = 0;
double sub_goal_local_y = 0;
double sub_goal_global_x = 0;
double sub_goal_global_y = 0;
ros::Time last_request;
double estimated_time = 0;
bool rviz_conops_callback_flg = false;
double LPF_gain = 0.02;
double orig_z = 0;
double prev_z = 0;

int COMEBACK_LANDING_CNT_THRES = 100;
int comeback_landing_cnt = 0;

double travel_dist = 0.0;
double tmp_travel_dist = 0.0;

enum FLIGHT_SERVICE_TYPE {
  TAKE_OFF = 1,
  LANDING,
  WPT_FOLLOWING,
  PATH_FLIGHT,
  HOVERING
};
std::shared_ptr<low_pass_filter> lpf;
void astar_callback(const nav_msgs::Path::ConstPtr &message) {
  astarpath = *message;
  if (astarpath.poses.size() >= look_ahead_ind) {
    sub_goal_local_x = astarpath.poses[look_ahead_ind - 1].pose.position.x;
    sub_goal_local_y = astarpath.poses[look_ahead_ind - 1].pose.position.y;
    sub_goal_global_x =
        current_pose.position.x +
        sub_goal_local_x * cos(current_heading_yaw_inGlobalCoor) -
        sub_goal_local_y * sin(current_heading_yaw_inGlobalCoor);
    //  global_y = current_pose.position.y +
    // body_vec_mag* sin(current_heading_yaw_inGlobalCoor);
    sub_goal_global_y =
        current_pose.position.y +
        sub_goal_local_x * sin(current_heading_yaw_inGlobalCoor) +
        sub_goal_local_y * cos(current_heading_yaw_inGlobalCoor);
  } else {
    sub_goal_global_x = rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                            .goal_pt_in_global.pose.position.x;
    sub_goal_global_y = rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                            .goal_pt_in_global.pose.position.y;
  }
  // vec_rviz_goals.push_back(rviz_goal_pose);
}
void z_max_callback(const std_msgs::Int8::ConstPtr &message) {
  max_z = message->data;
  // vec_rviz_goals.push_back(rviz_goal_pose);
}
void z_min_callback(const std_msgs::Int8::ConstPtr &message) {
  min_z = message->data;
  // vec_rviz_goals.push_back(rviz_goal_pose);
}
void x_max_callback(const std_msgs::Int8::ConstPtr &message) {
  max_x = message->data;
  // vec_rviz_goals.push_back(rviz_goal_pose);
}
void x_min_callback(const std_msgs::Int8::ConstPtr &message) {
  min_x = message->data;
  // vec_rviz_goals.push_back(rviz_goal_pose);
}
void y_max_callback(const std_msgs::Int8::ConstPtr &message) {
  max_y = message->data;
  // vec_rviz_goals.push_back(rviz_goal_pose);
}
void y_min_callback(const std_msgs::Int8::ConstPtr &message) {
  min_y = message->data;
  // vec_rviz_goals.push_back(rviz_goal_pose);
}
void rviz_goal_callback(const geometry_msgs::PoseStamped::ConstPtr &message) {
  rviz_goal_pose = *message;
  vec_rviz_goals.push_back(rviz_goal_pose);
  is_land = false;
  is_hover = false;
}
void land_callback(const std_msgs::Bool::ConstPtr &message) {
  is_land = message->data;
  // vec_rviz_goals.push_back(rviz_goal_pose);
}
void skip_callback(const std_msgs::Bool::ConstPtr &message) {
  // if (rviz_goal_list.behavior_n_goal_array[current_wpt_idx].mode ==
  //     WPT_FOLLOWING) {
  is_skip = message->data;
  // }
  // vec_rviz_goals.push_back(rviz_goal_pose);
}
void rviz_conops_callback(
    const kaist_drone_msgs::BehaviorNGoalArray::ConstPtr &message) {
  if (comeback_triggered == false) {
    rviz_goal_list = *message;
    current_wpt_idx = 0;
    // rviz_conops_callback_flg = true;
    last_request = ros::Time::now();
    dist_next_goal_to_robot =
        sqrt(pow(rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                         .goal_pt_in_global.pose.position.x -
                     current_pose.position.x,
                 2) +
             pow(rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                         .goal_pt_in_global.pose.position.y -
                     current_pose.position.y,
                 2) +
             pow(rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                         .goal_pt_in_global.pose.position.z -
                     current_pose.position.z,
                 2));
    estimated_time = dist_next_goal_to_robot / 0.5;
    prev_z = rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                 .goal_pt_in_global.pose.position.z;
    rviz_conops_callback_flg = true;
  }
}

void current_pose_callback(const nav_msgs::Odometry::ConstPtr &message) {
  current_pose.position.x = message->pose.pose.position.x;
  current_pose.position.y = message->pose.pose.position.y;
  current_pose.position.z = message->pose.pose.position.z;
  current_pose.orientation.x = message->pose.pose.orientation.x;
  current_pose.orientation.y = message->pose.pose.orientation.y;
  current_pose.orientation.z = message->pose.pose.orientation.z;
  current_pose.orientation.w = message->pose.pose.orientation.w;

  if (pose_callback_first_run) {
    past_pose = current_pose;
    pose_callback_first_run = false;
  }

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

  if (rviz_conops_callback_flg == true && comeback_triggered == false) {
    if (rviz_goal_list.behavior_n_goal_array[current_wpt_idx].mode ==
        rviz_goal_list.behavior_n_goal_array[current_wpt_idx].WPT_FOLLOWING) {
      double dist_diff =
          sqrt(pow(current_pose.position.x - past_pose.position.x, 2) +
               pow(current_pose.position.y - past_pose.position.y, 2) +
               pow(current_pose.position.z - past_pose.position.z, 2));

      tmp_travel_dist += dist_diff;
      if (tmp_travel_dist > 5.0) {
        tmp_travel_dist = 0.0;

        kaist_drone_msgs::BehaviorNGoal bNg;
        bNg.goal_pt_in_global.pose.position.x = current_pose.position.x;
        bNg.goal_pt_in_global.pose.position.y = current_pose.position.y;
        bNg.goal_pt_in_global.pose.position.z = current_pose.position.z;
        bNg.mode = bNg.WPT_FOLLOWING;

        backtracking_goal_list.behavior_n_goal_array.insert(
            backtracking_goal_list.behavior_n_goal_array.begin(), bNg);
      }
      travel_dist += dist_diff;
      if (travel_dist > COMEBACK_MAX_TRAVEL_RANGE) {
        comeback_triggered = true;

        kaist_drone_msgs::BehaviorNGoal bNg;
        bNg.goal_pt_in_global.pose.position.x = -2.0;
        bNg.goal_pt_in_global.pose.position.y = 0.0;
        bNg.goal_pt_in_global.pose.position.z = 0.0;
        bNg.mode = bNg.LAND;
        backtracking_goal_list.behavior_n_goal_array.push_back(bNg);

        // clear and then paste
        rviz_goal_list.behavior_n_goal_array.clear();
        current_wpt_idx = 0;
        rviz_goal_list = backtracking_goal_list;
      }
    }
  }

  if (sqrt(pow(current_pose.position.x, 2) + pow(current_pose.position.y, 2)) <
          3.0 &&
      current_pose.position.z < 0.3 &&
      rviz_goal_list.behavior_n_goal_array[current_wpt_idx].mode ==
          rviz_goal_list.behavior_n_goal_array[current_wpt_idx].LAND &&
      comeback_triggered == true) {

    comeback_landing_cnt++;

    if (comeback_landing_cnt > COMEBACK_LANDING_CNT_THRES) {
      comeback_triggered = false;
      backtracking_goal_list.behavior_n_goal_array.clear();
      rviz_goal_list.behavior_n_goal_array.clear();
    }
  }

  past_pose = current_pose;
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
  double spin_hz = 50; // hz
  ros::Rate r(spin_hz);

  bool random_wpt_generation_mode;
  string wpt_file_path;
  string wpt_topic_name;
  string odom_topic_name;
  string rviz_conops_topic_name;
  string rviz_goal_topic_name;
  string rviz_conops;

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

  double conop_takeoff_x, conop_takeoff_y, conop_takeoff_z,
      conop_flight_fixed_z;

  nh.getParam("random_generation", random_wpt_generation_mode);
  nh.getParam("predefined_wpt_file_path", wpt_file_path);
  nh.getParam("pub_wpt_topic_name", wpt_topic_name);
  nh.getParam("sub_odom_topic_name", odom_topic_name);
  nh.getParam("sub_rviz_conops_topic_name", rviz_conops_topic_name);
  nh.getParam("sub_rviz_goal_topic_name", rviz_goal_topic_name);
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
  nh.getParam("conop_takeoff_z", conop_takeoff_z);
  nh.getParam("conop_flight_fixed_z", conop_flight_fixed_z);
  nh.getParam("conops_rviz", rviz_conops);

  double lpf_cut_freq_hz;
  nh.param("height_command_lpf_cut_freq_hz", lpf_cut_freq_hz, 1.0);
  bool use_height_command_lpf;
  nh.param("use_height_command_lpf", use_height_command_lpf, false);

  last_request = ros::Time::now();

  ros::Subscriber rviz_goal_pose_subscriber =
      nh.subscribe(rviz_goal_topic_name, 10, rviz_goal_callback);

  if (spin_hz) {
    lpf =
        std::make_shared<low_pass_filter>(1.0 / spin_hz, lpf_cut_freq_hz, 0.0);
  } else {
    lpf = std::make_shared<low_pass_filter>(0.01, 10.0, 0.0);
  }

  vector<vector<double>> wpt_xy;
  // ros::Time last_request = ros::Time::now();

  if (rviz_conops == "conops_rviz") {
  } else {
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
          std::cout << "WPT RANDOM / GENERATION IN BODY COORDINATE"
                    << std::endl;
          std::cout << "WPT EMPTY" << std::endl;
          std::cout << "NODE END" << std::endl;
          return 0;
        } else {
          std::cout << "VERSION : RANDOM WPT / GENERATION IN BODY COORDINATE"
                    << std::endl;
          std::cout << "# OF WPT : " << wpt_xyz_global_coor.size() << std::endl;
          for (int wpt_idx = 0; wpt_idx < wpt_xyz_global_coor.size();
               wpt_idx++) {
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
          for (int wpt_idx = 0; wpt_idx < wpt_xyz_global_coor.size();
               wpt_idx++) {
            std::cout << "WPT " << wpt_idx << ": "
                      << " X=" << std::get<0>(wpt_xyz_global_coor[wpt_idx])
                      << " Y=" << std::get<1>(wpt_xyz_global_coor[wpt_idx])
                      << " Z=" << std::get<2>(wpt_xyz_global_coor[wpt_idx])
                      << std::endl;
          }
        }
      } else if (random_generation_coordination == "conops") {
        std::cout << "WPT RANDOM / GENERATION IN RVIZ GLOBAL COORDINATE"
                  << std::endl;
        std::cout << "TAKE OFF POINT : X "
                  << "current pose x"
                  << " " << std::endl;
        std::cout << "TAKE OFF POINT : Y "
                  << "current pose y"
                  << " " << std::endl;
        std::cout << "TAKE OFF POINT : Z " << conop_takeoff_z << " "
                  << std::endl;
        std::cout << "TAKE OFF POINT : FIXED ALTITUDE " << conop_flight_fixed_z
                  << " " << std::endl;
      } else {
        wpt_xyz_global_coor.clear();
        std::cout << "RANDOM WPT GENERATION COORDINATION NAME IS WRONG"
                  << std::endl;
        std::cout << "NODE END" << std::endl;
        return 0;
      }
    }
  }

  //////////////////////////////
  // PUBLISHER AND SUBSCRIBER //
  //////////////////////////////
  ros::Subscriber astar_sub =
      nh.subscribe("/scout/astar_path_vis", 1, astar_callback);
  ros::Subscriber z_min_sub = nh.subscribe("/minz", 10, z_min_callback);
  ros::Subscriber z_max_sub = nh.subscribe("/maxz", 10, z_max_callback);
  ros::Subscriber x_min_sub = nh.subscribe("/minx", 10, x_min_callback);
  ros::Subscriber x_max_sub = nh.subscribe("/maxx", 10, x_max_callback);
  ros::Subscriber y_min_sub = nh.subscribe("/miny", 10, y_min_callback);
  ros::Subscriber y_max_sub = nh.subscribe("/maxy", 10, y_max_callback);
  ros::Subscriber cur_pose_subscriber =
      nh.subscribe(odom_topic_name, 10, current_pose_callback);
  // ros::Subscriber rviz_goal_pose_subscriber =
  //     nh.subscribe(rviz_goal_topic_name, 10, rviz_goal_callback);
  ros::Subscriber rviz_conops_subscriber =
      nh.subscribe("/scout/behaviorNgoalArray", 10, rviz_conops_callback);
  ros::Subscriber land_sub = nh.subscribe("/is_landing", 1, land_callback);
  ros::Subscriber skip_sub = nh.subscribe("/is_skip", 1, skip_callback);

  // ros::Subscriber land_sub = nh.subscribe("/history", 1, land_callback);
  ros::Publisher sub_goal_vis =
      nh.advertise<visualization_msgs::MarkerArray>("/Sub_goal", 10);
  ros::Publisher local_goal_publisher =
      nh.advertise<kaist_drone_msgs::BehaviorNGoal>(wpt_topic_name, 10);

  if (rviz_conops != "conops_rviz") {
    while (ros::ok()) {
      if (random_generation_coordination != "conops") {
        if (current_wpt_idx == 0) {
          // /////////////
          // Take-off mode
          // /////////////
          double wpt_x = current_pose.position.x + 0.0;
          double wpt_y = current_pose.position.y + 0.0;
          double wpt_z = std::get<2>(wpt_xyz_global_coor[current_wpt_idx]);

          if (random_generation_coordination == "conops") {
            wpt_z = conop_flight_fixed_z;
          }

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

          if (wpt_z - current_pose.position.z < TAKE_OFF_THRES) {
            // /////////////
            // take-off done
            // /////////////
            current_wpt_idx++;
          }
        } else if (current_wpt_idx >= wpt_xyz_global_coor.size()) {
          // ////////////
          // Landing mode
          // ////////////
          if (random_generation_coordination != "conops") {
          } else {
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

            double dist_goal_to_robot =
                sqrt(pow(wpt_x - current_pose.position.x, 2) +
                     pow(wpt_y - current_pose.position.y, 2) +
                     pow(wpt_z - current_pose.position.z, 2));

            if (dist_goal_to_robot < goal_arrived_boundary) {
              // ///////////
              // WPT arrived
              // ///////////
              current_wpt_idx++;
            }
          }
          std::cout << " Travel info : " << current_wpt_idx << " / "
                    << wpt_xyz_global_coor.size() << std::endl;
        } else {
          // rviz goal pose sub
          if (current_wpt_idx == 0) {
            // /////////////
            // Take-off mode
            // /////////////
            double wpt_x = current_pose.position.x + 0.0;
            double wpt_y = current_pose.position.y + 0.0;
            double wpt_z = conop_takeoff_z;

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

            if (wpt_z - current_pose.position.z < TAKE_OFF_THRES) {
              // /////////////
              // take-off done
              // /////////////
              current_wpt_idx++;
            }
          } else {
            // /////////////
            // WPT following
            // /////////////
            double wpt_x = rviz_goal_pose.pose.position.x;
            double wpt_y = rviz_goal_pose.pose.position.y;
            double wpt_z = conop_flight_fixed_z;

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

            double dist_goal_to_robot =
                sqrt(pow(wpt_x - current_pose.position.x, 2) +
                     pow(wpt_y - current_pose.position.y, 2) +
                     pow(wpt_z - current_pose.position.z, 2));

            if (dist_goal_to_robot < goal_arrived_boundary) {
              // ///////////
              // WPT arrived
              // ///////////
              current_wpt_idx++;
            }
          }
          std::cout << " Travel info : " << current_wpt_idx << " / "
                    << "??" << std::endl;
        }

        ros::spinOnce();
        r.sleep();
      }
    }
  } else {
    while (ros::ok()) {
      if (rviz_conops_callback_flg == true) {
        kaist_drone_msgs::BehaviorNGoal msg;

        if (rviz_goal_list.behavior_n_goal_array[current_wpt_idx].mode ==
            msg.TAKEOFF) {
          geometry_msgs::PoseStamped pt_msg;
          pt_msg.header.stamp = ros::Time::now();
          pt_msg.header.frame_id = "odom";
          pt_msg.pose.position.x =
              rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                  .goal_pt_in_global.pose.position.x;
          pt_msg.pose.position.y =
              rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                  .goal_pt_in_global.pose.position.y;
          pt_msg.pose.position.z =
              rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                  .goal_pt_in_global.pose.position.z;
          msg.goal_pt_in_global = pt_msg;
          msg.mode = rviz_goal_list.behavior_n_goal_array[current_wpt_idx].mode;
          local_goal_publisher.publish(msg);

          if (pt_msg.pose.position.z - current_pose.position.z <
              TAKE_OFF_THRES) {
            // /////////////
            // take-off done
            // /////////////

            current_wpt_idx++;
            last_request = ros::Time::now();
            is_skip = false;

            if (rviz_goal_list.behavior_n_goal_array.size() - 1 <
                current_wpt_idx) {
              current_wpt_idx = rviz_goal_list.behavior_n_goal_array.size() - 1;
            }
            dist_next_goal_to_robot =
                sqrt(pow(rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                                 .goal_pt_in_global.pose.position.x -
                             current_pose.position.x,
                         2) +
                     pow(rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                                 .goal_pt_in_global.pose.position.y -
                             current_pose.position.y,
                         2) +
                     pow(rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                                 .goal_pt_in_global.pose.position.z -
                             current_pose.position.z,
                         2));
            estimated_time = dist_next_goal_to_robot / 0.5;
            prev_z = rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                         .goal_pt_in_global.pose.position.z;
          }

          // Set LPF flag
          // lpf->setFirstCallTrue();
        } else {
          geometry_msgs::PoseStamped pt_msg;
          geometry_msgs::PoseStamped sub_pt_msg;

          double local_x, local_y, local_z;
          pt_msg.header.stamp = ros::Time::now();
          pt_msg.header.frame_id = "odom";

          sub_pt_msg.header.stamp = ros::Time::now();
          sub_pt_msg.header.frame_id = "odom";

          pt_msg.pose.position.x =
              rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                  .goal_pt_in_global.pose.position.x;
          pt_msg.pose.position.y =
              rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                  .goal_pt_in_global.pose.position.y;
          pt_msg.pose.position.z =
              (rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                   .goal_pt_in_global.pose.position.z);

          sub_pt_msg.pose.position.x = sub_goal_global_x;
          sub_pt_msg.pose.position.y = sub_goal_global_y;
          sub_pt_msg.pose.position.z =
              (rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                   .goal_pt_in_global.pose.position.z);

          local_x = cos(-current_heading_yaw_inGlobalCoor) *
                        (pt_msg.pose.position.x - current_pose.position.x) -
                    sin(-current_heading_yaw_inGlobalCoor) *
                        (pt_msg.pose.position.y - current_pose.position.y);
          local_y = sin(-current_heading_yaw_inGlobalCoor) *
                        (pt_msg.pose.position.x - current_pose.position.x) +
                    cos(-current_heading_yaw_inGlobalCoor) *
                        (pt_msg.pose.position.y - current_pose.position.y);
          local_z = pt_msg.pose.position.z - current_pose.position.z;

          if ((pt_msg.pose.position.z <= max_z) &&
              (pt_msg.pose.position.z >= min_z)) {
          } else {
            pt_msg.pose.position.z = (max_z + min_z) / 2;
          }
          prev_z = pt_msg.pose.position.z;
          msg.goal_pt_in_global = pt_msg;
          double dist_goal_to_robot =
              sqrt(pow(pt_msg.pose.position.x - current_pose.position.x, 2) +
                   pow(pt_msg.pose.position.y - current_pose.position.y, 2));
          msg.mode = rviz_goal_list.behavior_n_goal_array[current_wpt_idx].mode;
          if ((msg.mode == msg.WPT_FOLLOWING) && (is_hover == true)) {
            msg.mode = msg.HOVER;
          }
          if ((is_land == true) && (msg.mode != msg.ESTOP)) {
            msg.mode = msg.IDLE;
          }

          local_goal_publisher.publish(msg);

          if ((dist_goal_to_robot < goal_arrived_boundary) ||
              (is_skip == true) ||
              ((ros::Time::now() - last_request) >
               ros::Duration(estimated_time))) {
            // WPT arrived
            // ///////////
            current_wpt_idx++;
            last_request = ros::Time::now();
            is_skip = false;

            if (rviz_goal_list.behavior_n_goal_array.size() - 1 <
                current_wpt_idx) {
              current_wpt_idx = rviz_goal_list.behavior_n_goal_array.size() - 1;
              is_hover = true;
            }
            dist_next_goal_to_robot =
                sqrt(pow(rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                                 .goal_pt_in_global.pose.position.x -
                             current_pose.position.x,
                         2) +
                     pow(rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                                 .goal_pt_in_global.pose.position.y -
                             current_pose.position.y,
                         2) +
                     pow(rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                                 .goal_pt_in_global.pose.position.z -
                             current_pose.position.z,
                         2));
            estimated_time = dist_next_goal_to_robot / 0.5;
            prev_z = rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                         .goal_pt_in_global.pose.position.z;

          } else {
            is_hover = false;
          }
        }
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        geometry_msgs::Point p;
        p.x = sub_goal_global_x;
        p.y = sub_goal_global_y;
        p.z = rviz_goal_list.behavior_n_goal_array[current_wpt_idx]
                  .goal_pt_in_global.pose.position.z;

        marker.points.push_back(p);
        marker.header.frame_id = "/odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "spheres";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;

        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;

        // POINTS markers use x and y scale for width/height respectively
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;

        // Points are green
        // marker
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
        marker_array.markers.push_back(marker);
        sub_goal_vis.publish(marker_array);
        // is_land = false;
      } else {
      }
      std::cout << (ros::Duration(estimated_time) -
                    (ros::Time::now() - last_request))
                << " second remain" << std::endl;
      ros::spinOnce();
      r.sleep();

      /* code */
    }
  }

  return 0;
}