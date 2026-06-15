#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <librealsense2/rs.hpp>
#include <ros/ros.h>

class RealsenseWanderObstacleAvoidance {
public:
    RealsenseWanderObstacleAvoidance()
        : nh_()
        , private_nh_("~")
        , rng_(std::random_device()())
        , turn_dir_distribution_(0, 1)
        , cruise_duration_distribution_(3.0, 8.0)
        , turn_duration_distribution_(1.0, 2.5)
    {
        private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
        private_nh_.param<std::string>("serial_no", serial_no_, "");
        private_nh_.param("depth_width", depth_width_, 424);
        private_nh_.param("depth_height", depth_height_, 240);
        private_nh_.param("depth_fps", depth_fps_, 15);
        private_nh_.param("control_rate", control_rate_, 10.0);

        private_nh_.param("linear_speed", linear_speed_, 0.10);
        private_nh_.param("angular_speed", angular_speed_, 0.50);
        private_nh_.param("escape_angular_speed", escape_angular_speed_, 0.80);
        private_nh_.param("forward_clearance", forward_clearance_, 0.40);
        private_nh_.param("side_clearance", side_clearance_, 0.32);
        private_nh_.param("emergency_clearance", emergency_clearance_, 0.30);
        private_nh_.param("min_valid_depth", min_valid_depth_, 0.18);
        private_nh_.param("max_valid_depth", max_valid_depth_, 3.50);

        private_nh_.param("roi_top_ratio", roi_top_ratio_, 0.35);
        private_nh_.param("roi_bottom_ratio", roi_bottom_ratio_, 0.75);
        private_nh_.param("left_start_ratio", left_start_ratio_, 0.08);
        private_nh_.param("left_end_ratio", left_end_ratio_, 0.34);
        private_nh_.param("center_start_ratio", center_start_ratio_, 0.38);
        private_nh_.param("center_end_ratio", center_end_ratio_, 0.62);
        private_nh_.param("right_start_ratio", right_start_ratio_, 0.66);
        private_nh_.param("right_end_ratio", right_end_ratio_, 0.92);

        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);

        configurePipeline();
        random_cruise_until_ = ros::Time::now() + ros::Duration(sampleCruiseDuration());
        random_turn_until_ = ros::Time(0);
    }

    ~RealsenseWanderObstacleAvoidance()
    {
        publishStop();
        try {
            pipeline_.stop();
        } catch (...) {
        }
    }

    void run()
    {
        ros::Rate rate(control_rate_);
        ROS_INFO("realsense_wander_obstacle_avoidance is running.");

        while (ros::ok()) {
            geometry_msgs::Twist cmd = computeCommand();
            cmd_pub_.publish(cmd);
            ros::spinOnce();
            rate.sleep();
        }

        publishStop();
    }

private:
    struct SectorStats {
        float center;
        float left;
        float right;
    };

    void configurePipeline()
    {
        rs2::context ctx;
        rs2::device_list devices = ctx.query_devices();
        if (devices.size() == 0) {
            throw std::runtime_error("No RealSense device detected.");
        }

        bool matched_device = false;
        for (rs2::device dev : devices) {
            std::string name = dev.supports(RS2_CAMERA_INFO_NAME) ? dev.get_info(RS2_CAMERA_INFO_NAME) : "unknown";
            std::string serial = dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) ? dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) : "";
            ROS_INFO("Detected RealSense device: %s serial=%s", name.c_str(), serial.c_str());
            if (serial_no_.empty() || serial == serial_no_) {
                serial_no_ = serial;
                matched_device = true;
                break;
            }
        }

        if (!matched_device) {
            throw std::runtime_error("Requested RealSense serial was not found: " + serial_no_);
        }

        config_.enable_device(serial_no_);
        config_.enable_stream(RS2_STREAM_DEPTH, depth_width_, depth_height_, RS2_FORMAT_Z16, depth_fps_);

        rs2::pipeline_profile profile = pipeline_.start(config_);
        rs2::device active_device = profile.get_device();
        std::string active_name = active_device.supports(RS2_CAMERA_INFO_NAME) ? active_device.get_info(RS2_CAMERA_INFO_NAME) : "unknown";
        ROS_INFO("Started RealSense depth stream: %s serial=%s %dx%d@%d",
                 active_name.c_str(), serial_no_.c_str(), depth_width_, depth_height_, depth_fps_);
    }

    geometry_msgs::Twist computeCommand()
    {
        try {
            rs2::frameset frames = pipeline_.wait_for_frames(1000);
            rs2::depth_frame depth = frames.get_depth_frame();
            if (!depth) {
                ROS_WARN_THROTTLE(2.0, "No RealSense depth frame received.");
                return stopCommand();
            }

            SectorStats sector_stats = computeSectorStats(depth);
            return decideCommand(sector_stats);
        } catch (const rs2::error& exc) {
            ROS_WARN_THROTTLE(2.0, "RealSense error: %s", exc.what());
            return stopCommand();
        } catch (const std::exception& exc) {
            ROS_WARN_THROTTLE(2.0, "Depth processing error: %s", exc.what());
            return stopCommand();
        }
    }

    SectorStats computeSectorStats(const rs2::depth_frame& depth)
    {
        const int width = depth.get_width();
        const int height = depth.get_height();

        const int row_start = clampIndex(static_cast<int>(height * roi_top_ratio_), 0, height - 1);
        const int row_end = clampIndex(static_cast<int>(height * roi_bottom_ratio_), row_start + 1, height);

        const int left_start = clampIndex(static_cast<int>(width * left_start_ratio_), 0, width - 1);
        const int left_end = clampIndex(static_cast<int>(width * left_end_ratio_), left_start + 1, width);
        const int center_start = clampIndex(static_cast<int>(width * center_start_ratio_), 0, width - 1);
        const int center_end = clampIndex(static_cast<int>(width * center_end_ratio_), center_start + 1, width);
        const int right_start = clampIndex(static_cast<int>(width * right_start_ratio_), 0, width - 1);
        const int right_end = clampIndex(static_cast<int>(width * right_end_ratio_), right_start + 1, width);

        std::vector<float> left_values;
        std::vector<float> center_values;
        std::vector<float> right_values;
        left_values.reserve((row_end - row_start) * (left_end - left_start));
        center_values.reserve((row_end - row_start) * (center_end - center_start));
        right_values.reserve((row_end - row_start) * (right_end - right_start));

        for (int y = row_start; y < row_end; ++y) {
            for (int x = left_start; x < left_end; ++x) {
                maybePushDepth(depth.get_distance(x, y), left_values);
            }
            for (int x = center_start; x < center_end; ++x) {
                maybePushDepth(depth.get_distance(x, y), center_values);
            }
            for (int x = right_start; x < right_end; ++x) {
                maybePushDepth(depth.get_distance(x, y), right_values);
            }
        }

        SectorStats stats;
        stats.left = percentile(left_values, 0.10f);
        stats.center = percentile(center_values, 0.10f);
        stats.right = percentile(right_values, 0.10f);
        return stats;
    }

    void maybePushDepth(float distance_m, std::vector<float>& output) const
    {
        if (!std::isfinite(distance_m)) {
            return;
        }
        if (distance_m < min_valid_depth_ || distance_m > max_valid_depth_) {
            return;
        }
        output.push_back(distance_m);
    }

    float percentile(std::vector<float>& values, float probability) const
    {
        if (values.empty()) {
            return std::numeric_limits<float>::infinity();
        }

        const std::size_t index = std::min<std::size_t>(
            values.size() - 1,
            static_cast<std::size_t>(probability * static_cast<float>(values.size() - 1)));
        std::nth_element(values.begin(), values.begin() + index, values.end());
        return values[index];
    }

    geometry_msgs::Twist decideCommand(const SectorStats& sector_stats)
    {
        if (sector_stats.center < emergency_clearance_) {
            turn_direction_ = selectTurnDirection(sector_stats.left, sector_stats.right);
            random_turn_until_ = ros::Time::now() + ros::Duration(sampleTurnDuration());
            random_cruise_until_ = random_turn_until_ + ros::Duration(sampleCruiseDuration());
            return makeCommand(0.0, turn_direction_ * escape_angular_speed_);
        }

        if (sector_stats.center < forward_clearance_
            || sector_stats.left < side_clearance_
            || sector_stats.right < side_clearance_) {
            turn_direction_ = selectTurnDirection(sector_stats.left, sector_stats.right);
            random_turn_until_ = ros::Time::now() + ros::Duration(sampleTurnDuration());
            random_cruise_until_ = random_turn_until_ + ros::Duration(sampleCruiseDuration());
            return makeCommand(0.02, turn_direction_ * angular_speed_);
        }

        maybeStartRandomTurn();
        if (ros::Time::now() < random_turn_until_) {
            return makeCommand(0.04, turn_direction_ * (0.7 * angular_speed_));
        }

        const double steer_bias = clamp((sector_stats.right - sector_stats.left) * 0.45, -0.20, 0.20);
        return makeCommand(linear_speed_, steer_bias);
    }

    void maybeStartRandomTurn()
    {
        const ros::Time now = ros::Time::now();
        if (now >= random_cruise_until_ && now >= random_turn_until_) {
            turn_direction_ = turn_dir_distribution_(rng_) == 0 ? -1.0 : 1.0;
            random_turn_until_ = now + ros::Duration(sampleTurnDuration());
            random_cruise_until_ = random_turn_until_ + ros::Duration(sampleCruiseDuration());
        }
    }

    double selectTurnDirection(float left, float right)
    {
        if (left > right + 0.05f) {
            return 1.0;
        }
        if (right > left + 0.05f) {
            return -1.0;
        }
        return turn_dir_distribution_(rng_) == 0 ? -1.0 : 1.0;
    }

    geometry_msgs::Twist makeCommand(double linear_x, double angular_z) const
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = linear_x;
        cmd.angular.z = angular_z;
        return cmd;
    }

    geometry_msgs::Twist stopCommand() const
    {
        return makeCommand(0.0, 0.0);
    }

    void publishStop()
    {
        cmd_pub_.publish(stopCommand());
    }

    double sampleCruiseDuration()
    {
        return cruise_duration_distribution_(rng_);
    }

    double sampleTurnDuration()
    {
        return turn_duration_distribution_(rng_);
    }

    static int clampIndex(int value, int lower, int upper)
    {
        return std::max(lower, std::min(value, upper));
    }

    static double clamp(double value, double lower, double upper)
    {
        return std::max(lower, std::min(value, upper));
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher cmd_pub_;

    rs2::pipeline pipeline_;
    rs2::config config_;

    std::string cmd_vel_topic_;
    std::string serial_no_;
    int depth_width_;
    int depth_height_;
    int depth_fps_;
    double control_rate_;

    double linear_speed_;
    double angular_speed_;
    double escape_angular_speed_;
    double forward_clearance_;
    double side_clearance_;
    double emergency_clearance_;
    double min_valid_depth_;
    double max_valid_depth_;

    double roi_top_ratio_;
    double roi_bottom_ratio_;
    double left_start_ratio_;
    double left_end_ratio_;
    double center_start_ratio_;
    double center_end_ratio_;
    double right_start_ratio_;
    double right_end_ratio_;

    std::mt19937 rng_;
    std::uniform_int_distribution<int> turn_dir_distribution_;
    std::uniform_real_distribution<double> cruise_duration_distribution_;
    std::uniform_real_distribution<double> turn_duration_distribution_;
    double turn_direction_{1.0};
    ros::Time random_turn_until_;
    ros::Time random_cruise_until_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_wander_obstacle_avoidance");

    try {
        RealsenseWanderObstacleAvoidance node;
        node.run();
        return 0;
    } catch (const std::exception& exc) {
        ROS_FATAL("Failed to start realsense_wander_obstacle_avoidance: %s", exc.what());
        return 1;
    }
}
