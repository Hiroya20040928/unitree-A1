#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class TebViaTeleop
{
public:
  TebViaTeleop()
    : nh_()
    , private_nh_("~")
    , move_base_client_(move_base_action_name(), true)
  {
    private_nh_.param<std::string>("global_frame", global_frame_, "slamware_map");
    private_nh_.param<std::string>("base_frame", base_frame_, "base_link");
    private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/teb_via_teleop/cmd_vel");
    private_nh_.param<std::string>("via_points_topic", via_points_topic_, "/move_base/TebLocalPlannerROS/via_points");
    private_nh_.param<std::string>("move_base_action", move_base_action_, "/move_base");

    private_nh_.param("control_rate", control_rate_, 10.0);
    private_nh_.param("command_timeout", command_timeout_, 0.35);
    private_nh_.param("horizon_time", horizon_time_, 2.2);
    private_nh_.param("sample_dt", sample_dt_, 0.2);
    private_nh_.param("max_lookahead_distance", max_lookahead_distance_, 1.2);
    private_nh_.param("max_linear_speed", max_linear_speed_, 0.28);
    private_nh_.param("max_angular_speed", max_angular_speed_, 0.9);
    private_nh_.param("goal_update_dist", goal_update_dist_, 0.10);
    private_nh_.param("goal_update_yaw", goal_update_yaw_, 0.20);
    private_nh_.param("goal_refresh_time", goal_refresh_time_, 0.75);
    private_nh_.param("min_translation_for_path", min_translation_for_path_, 0.03);
    private_nh_.param("allow_reverse", allow_reverse_, false);

    cmd_sub_ = nh_.subscribe(cmd_vel_topic_, 1, &TebViaTeleop::cmdCallback, this);
    via_points_pub_ = nh_.advertise<nav_msgs::Path>(via_points_topic_, 1, true);
    debug_path_pub_ = nh_.advertise<nav_msgs::Path>("teb_via_teleop/debug_path", 1, true);

    last_cmd_time_ = ros::Time(0);
    last_goal_sent_time_ = ros::Time(0);
    goal_active_ = false;
    path_active_ = false;
    have_last_goal_pose_ = false;

    ROS_INFO("teb_via_teleop configured:"
             " cmd_vel_topic=%s via_points_topic=%s move_base_action=%s global_frame=%s base_frame=%s",
             cmd_vel_topic_.c_str(), via_points_topic_.c_str(), move_base_action_.c_str(),
             global_frame_.c_str(), base_frame_.c_str());
  }

  void spin()
  {
    ros::Rate rate(control_rate_);
    while (ros::ok()) {
      ros::spinOnce();
      update();
      rate.sleep();
    }

    publishEmptyViaPoints();
    cancelGoal();
  }

private:
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  std::string move_base_action_name() const
  {
    std::string action_name;
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("move_base_action", action_name, "/move_base");
    return action_name;
  }

  void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    last_cmd_ = *msg;
    last_cmd_time_ = ros::Time::now();
  }

  void update()
  {
    tf::StampedTransform robot_tf;
    if (!lookupRobotTransform(robot_tf)) {
      publishEmptyViaPoints();
      cancelGoal();
      return;
    }

    if (!commandActive()) {
      publishEmptyViaPoints();
      cancelGoal();
      return;
    }

    nav_msgs::Path via_path = buildViaPath(robot_tf);
    via_points_pub_.publish(via_path);
    debug_path_pub_.publish(via_path);
    path_active_ = !via_path.poses.empty();

    if (!via_path.poses.empty()) {
      maybeSendGoal(via_path.poses.back());
    }
  }

  bool lookupRobotTransform(tf::StampedTransform& robot_tf)
  {
    try {
      tf_listener_.lookupTransform(global_frame_, base_frame_, ros::Time(0), robot_tf);
      return true;
    } catch (const tf::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0, "teb_via_teleop tf lookup failed (%s -> %s): %s",
                        global_frame_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool commandActive() const
  {
    if (last_cmd_time_.isZero()) {
      return false;
    }
    if ((ros::Time::now() - last_cmd_time_).toSec() > command_timeout_) {
      return false;
    }
    return std::fabs(last_cmd_.linear.x) > 1e-3 || std::fabs(last_cmd_.angular.z) > 1e-3;
  }

  nav_msgs::Path buildViaPath(const tf::StampedTransform& robot_tf)
  {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = global_frame_;

    double linear = clamp(last_cmd_.linear.x, allow_reverse_ ? -max_linear_speed_ : 0.0, max_linear_speed_);
    double angular = clamp(last_cmd_.angular.z, -max_angular_speed_, max_angular_speed_);

    double x = robot_tf.getOrigin().x();
    double y = robot_tf.getOrigin().y();
    double yaw = tf::getYaw(robot_tf.getRotation());
    double travelled = 0.0;

    const int samples = std::max(1, static_cast<int>(std::ceil(horizon_time_ / sample_dt_)));
    if (std::fabs(linear) < min_translation_for_path_) {
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw + angular * horizon_time_);
      path.poses.push_back(pose);
      return path;
    }

    for (int i = 0; i < samples; ++i) {
      yaw += angular * sample_dt_;
      x += linear * std::cos(yaw) * sample_dt_;
      y += linear * std::sin(yaw) * sample_dt_;
      travelled += std::fabs(linear) * sample_dt_;

      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      path.poses.push_back(pose);

      if (travelled >= max_lookahead_distance_) {
        break;
      }
    }

    return path;
  }

  void maybeSendGoal(const geometry_msgs::PoseStamped& pose)
  {
    if (!move_base_client_.waitForServer(ros::Duration(0.0))) {
      ROS_WARN_THROTTLE(2.0, "teb_via_teleop is waiting for move_base action server %s", move_base_action_.c_str());
      return;
    }

    if (!shouldUpdateGoal(pose)) {
      return;
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = pose;
    goal.target_pose.header.stamp = ros::Time::now();
    move_base_client_.sendGoal(goal);

    last_goal_pose_ = pose;
    last_goal_sent_time_ = ros::Time::now();
    goal_active_ = true;
    have_last_goal_pose_ = true;
  }

  bool shouldUpdateGoal(const geometry_msgs::PoseStamped& pose) const
  {
    if (!have_last_goal_pose_) {
      return true;
    }

    const double dx = pose.pose.position.x - last_goal_pose_.pose.position.x;
    const double dy = pose.pose.position.y - last_goal_pose_.pose.position.y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    const double yaw = tf::getYaw(pose.pose.orientation);
    const double last_yaw = tf::getYaw(last_goal_pose_.pose.orientation);
    const double yaw_diff = shortestAngularDistance(last_yaw, yaw);
    const double since_last_goal = (ros::Time::now() - last_goal_sent_time_).toSec();

    return dist >= goal_update_dist_
        || std::fabs(yaw_diff) >= goal_update_yaw_
        || since_last_goal >= goal_refresh_time_;
  }

  void publishEmptyViaPoints()
  {
    if (!path_active_) {
      return;
    }

    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = global_frame_;
    via_points_pub_.publish(path);
    debug_path_pub_.publish(path);
    path_active_ = false;
  }

  void cancelGoal()
  {
    if (!goal_active_) {
      return;
    }
    if (move_base_client_.waitForServer(ros::Duration(0.0))) {
      move_base_client_.cancelAllGoals();
    }
    goal_active_ = false;
    have_last_goal_pose_ = false;
  }

  static double clamp(double value, double lower, double upper)
  {
    return std::max(lower, std::min(value, upper));
  }

  static double shortestAngularDistance(double from, double to)
  {
    double delta = std::fmod(to - from + M_PI, 2.0 * M_PI);
    if (delta < 0.0) {
      delta += 2.0 * M_PI;
    }
    return delta - M_PI;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cmd_sub_;
  ros::Publisher via_points_pub_;
  ros::Publisher debug_path_pub_;
  MoveBaseClient move_base_client_;
  tf::TransformListener tf_listener_;

  std::string global_frame_;
  std::string base_frame_;
  std::string cmd_vel_topic_;
  std::string via_points_topic_;
  std::string move_base_action_;

  double control_rate_;
  double command_timeout_;
  double horizon_time_;
  double sample_dt_;
  double max_lookahead_distance_;
  double max_linear_speed_;
  double max_angular_speed_;
  double goal_update_dist_;
  double goal_update_yaw_;
  double goal_refresh_time_;
  double min_translation_for_path_;
  bool allow_reverse_;

  geometry_msgs::Twist last_cmd_;
  ros::Time last_cmd_time_;
  ros::Time last_goal_sent_time_;
  geometry_msgs::PoseStamped last_goal_pose_;
  bool goal_active_;
  bool path_active_;
  bool have_last_goal_pose_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teb_via_teleop");
  TebViaTeleop node;
  node.spin();
  return 0;
}
