#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std::chrono_literals;

// Dynamixel Control Table Addresses
#define ADDR_OPERATING_MODE    11
#define ADDR_TORQUE_ENABLE     64
#define ADDR_LED_RED           65
#define ADDR_GOAL_VELOCITY     104
#define ADDR_PROFILE_ACCEL     108
#define ADDR_PRESENT_VELOCITY  128
#define ADDR_PRESENT_POSITION  132

#define LEN_GOAL_VELOCITY      4
#define LEN_PRESENT_VELOCITY   4
#define LEN_PRESENT_POSITION   4

#define PROTOCOL_VERSION       2.0

class PinkyBringup : public rclcpp::Node
{
public:
    PinkyBringup() : Node("pinky_bringup_cpp")
    {
        // Parameters
        this->declare_parameter<std::string>("port_name", "/dev/ttyAMA4");
        this->declare_parameter<int>("baudrate", 1000000);
        this->declare_parameter<std::vector<int64_t>>("dxl_ids", {1, 2});
        this->declare_parameter<double>("wheel_radius", 0.028);
        this->declare_parameter<double>("wheel_base", 0.0961);
        this->declare_parameter<int>("pulses_per_rot", 4096);
        this->declare_parameter<double>("control_rate", 50.0); // Default 50Hz
        this->declare_parameter<double>("low_battery_threshold", 6.8);
        this->declare_parameter<bool>("publish_tf", true);

        port_name_ = this->get_parameter("port_name").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();
        dxl_ids_ = this->get_parameter("dxl_ids").as_integer_array();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        pulses_per_rot_ = this->get_parameter("pulses_per_rot").as_int();
        control_rate_ = this->get_parameter("control_rate").as_double();
        low_battery_threshold_ = this->get_parameter("low_battery_threshold").as_double();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();

        circumference_ = 2.0 * M_PI * wheel_radius_;
        rpm_to_value_scale_ = 1.0 / 0.229;
        rpm_to_rads_ = 2.0 * M_PI / 60.0;

        // Initialize Dynamixel
        portHandler_ = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if (!portHandler_->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", port_name_.c_str());
            throw std::runtime_error("Failed to open port");
        }

        if (!portHandler_->setBaudRate(baudrate_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate: %d", baudrate_);
            throw std::runtime_error("Failed to set baudrate");
        }

        groupSyncWrite_ = std::make_unique<dynamixel::GroupSyncWrite>(portHandler_, packetHandler_, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY);
        groupBulkRead_ = std::make_unique<dynamixel::GroupBulkRead>(portHandler_, packetHandler_);

        initialize_motors();

        // ROS Interfaces
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "battery/voltage", 10, std::bind(&PinkyBringup::battery_callback, this, std::placeholders::_1));
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&PinkyBringup::twist_callback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Initial Feedback
        if (!read_feedback(last_rpm_l_, last_rpm_r_, last_encoder_l_, last_encoder_r_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read initial encoder position! Shutting down.");
            throw std::runtime_error("Failed to read initial encoder position");
        }
        RCLCPP_INFO(this->get_logger(), "Initial Encoder read: L=%d, R=%d", last_encoder_l_, last_encoder_r_);

        // Timer
        auto period = std::chrono::duration<double>(1.0 / control_rate_);
        timer_ = this->create_wall_timer(period, std::bind(&PinkyBringup::timer_callback, this));

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Pinky Bringup C++ Node started at %.1f Hz", control_rate_);
    }

    ~PinkyBringup()
    {
        set_double_rpm(0, 0);
        std::this_thread::sleep_for(100ms);
        for (auto id : dxl_ids_) {
            packetHandler_->write1ByteTxRx(portHandler_, (uint8_t)id, ADDR_TORQUE_ENABLE, 0);
            packetHandler_->write1ByteTxRx(portHandler_, (uint8_t)id, ADDR_LED_RED, 0);
        }
        portHandler_->closePort();
    }

private:
    void initialize_motors()
    {
        for (auto id : dxl_ids_) {
            packetHandler_->reboot(portHandler_, (uint8_t)id);
            std::this_thread::sleep_for(500ms);

            packetHandler_->write1ByteTxRx(portHandler_, (uint8_t)id, ADDR_OPERATING_MODE, 1); // Velocity Control Mode
            packetHandler_->write4ByteTxRx(portHandler_, (uint8_t)id, ADDR_PROFILE_ACCEL, 200);
            packetHandler_->write1ByteTxRx(portHandler_, (uint8_t)id, ADDR_TORQUE_ENABLE, 1);
            packetHandler_->write1ByteTxRx(portHandler_, (uint8_t)id, ADDR_LED_RED, 1);
        }
    }

    bool set_double_rpm(double rpm_l, double rpm_r)
    {
        groupSyncWrite_->clearParam();
        
        int32_t val_l = static_cast<int32_t>(rpm_l * rpm_to_value_scale_);
        int32_t val_r = static_cast<int32_t>(rpm_r * rpm_to_value_scale_);

        uint8_t param_l[4], param_r[4];
        param_l[0] = DXL_LOBYTE(DXL_LOWORD(val_l));
        param_l[1] = DXL_HIBYTE(DXL_LOWORD(val_l));
        param_l[2] = DXL_LOBYTE(DXL_HIWORD(val_l));
        param_l[3] = DXL_HIBYTE(DXL_HIWORD(val_l));

        param_r[0] = DXL_LOBYTE(DXL_LOWORD(val_r));
        param_r[1] = DXL_HIBYTE(DXL_LOWORD(val_r));
        param_r[2] = DXL_LOBYTE(DXL_HIWORD(val_r));
        param_r[3] = DXL_HIBYTE(DXL_HIWORD(val_r));

        groupSyncWrite_->addParam((uint8_t)dxl_ids_[0], param_l);
        groupSyncWrite_->addParam((uint8_t)dxl_ids_[1], param_r);

        return groupSyncWrite_->txPacket() == COMM_SUCCESS;
    }

    bool read_feedback(double &rpm_l, double &rpm_r, int32_t &pos_l, int32_t &pos_r)
    {
        groupBulkRead_->clearParam();
        uint16_t read_len = LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION;

        for (auto id : dxl_ids_) {
            groupBulkRead_->addParam((uint8_t)id, ADDR_PRESENT_VELOCITY, read_len);
        }

        if (groupBulkRead_->txRxPacket() != COMM_SUCCESS) return false;

        uint8_t id_l = (uint8_t)dxl_ids_[0];
        uint8_t id_r = (uint8_t)dxl_ids_[1];

        if (!groupBulkRead_->isAvailable(id_l, ADDR_PRESENT_VELOCITY, read_len) ||
            !groupBulkRead_->isAvailable(id_r, ADDR_PRESENT_VELOCITY, read_len)) return false;

        int32_t vel_raw_l = groupBulkRead_->getData(id_l, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
        pos_l = groupBulkRead_->getData(id_l, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        int32_t vel_raw_r = groupBulkRead_->getData(id_r, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
        pos_r = groupBulkRead_->getData(id_r, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

        rpm_l = static_cast<double>(vel_raw_l) / rpm_to_value_scale_;
        rpm_r = static_cast<double>(vel_raw_r) / rpm_to_value_scale_;

        return true;
    }

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;

        double v_l = linear_x - (angular_z * wheel_base_ / 2.0);
        double v_r = linear_x + (angular_z * wheel_base_ / 2.0);

        double rpm_l = (v_l / wheel_radius_) * 60.0 / (2.0 * M_PI);
        double rpm_r = -(v_r / wheel_radius_) * 60.0 / (2.0 * M_PI);

        // Limit RPM
        double max_rpm = 100.0;
        double current_max = std::max(std::abs(rpm_l), std::abs(rpm_r));
        if (current_max > max_rpm) {
            double scale = max_rpm / current_max;
            rpm_l *= scale;
            rpm_r *= scale;
        }

        set_double_rpm(rpm_l, rpm_r);
    }

    void battery_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (msg->data <= low_battery_threshold_) {
            RCLCPP_WARN(this->get_logger(), "!!! LOW BATTERY WARNING !!! Voltage: %.2fV", msg->data);
        }
    }

    void timer_callback()
    {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0) return;

        double rpm_l, rpm_r;
        int32_t encoder_l, encoder_r;

        if (!read_feedback(rpm_l, rpm_r, encoder_l, encoder_r)) {
            RCLCPP_WARN(this->get_logger(), "Failed to read motor data. Skipping update cycle.");
            return;
        }

        int32_t delta_l = encoder_l - last_encoder_l_;
        int32_t delta_r = -(encoder_r - last_encoder_r_);

        last_encoder_l_ = encoder_l;
        last_encoder_r_ = encoder_r;

        double dist_l = (static_cast<double>(delta_l) / pulses_per_rot_) * circumference_;
        double dist_r = (static_cast<double>(delta_r) / pulses_per_rot_) * circumference_;

        double delta_distance = (dist_r + dist_l) / 2.0;
        double delta_theta = (dist_r - dist_l) / wheel_base_;

        theta_ += delta_theta;
        x_ += delta_distance * std::cos(theta_);
        y_ += delta_distance * std::sin(theta_);

        double v_x = delta_distance / dt;
        double v_th = delta_theta / dt;

        publish_odom(current_time, v_x, v_th);
        publish_joint_states(current_time, rpm_l, rpm_r, encoder_l, encoder_r);

        last_time_ = current_time;
    }

    void publish_odom(const rclcpp::Time &time, double v_x, double v_th)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        // TF
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = time;
            t.header.frame_id = "odom";
            t.child_frame_id = "base_footprint";
            t.transform.translation.x = x_;
            t.transform.translation.y = y_;
            t.transform.rotation = tf2::toMsg(q);
            tf_broadcaster_->sendTransform(t);
        }

        // Odom
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation = tf2::toMsg(q);
        odom.twist.twist.linear.x = v_x;
        odom.twist.twist.angular.z = v_th;
        odom_pub_->publish(odom);
    }

    void publish_joint_states(const rclcpp::Time &time, double rpm_l, double rpm_r, int32_t encoder_l, int32_t encoder_r)
    {
        sensor_msgs::msg::JointState js;
        js.header.stamp = time;
        js.name = {"left_wheel_joint", "right_wheel_joint"};
        
        double pos_l = (static_cast<double>(encoder_l) / pulses_per_rot_) * (2.0 * M_PI);
        double pos_r = (static_cast<double>(encoder_r) / pulses_per_rot_) * (2.0 * M_PI);
        
        js.position = {pos_l, pos_r};
        js.velocity = {rpm_l * rpm_to_rads_, rpm_r * rpm_to_rads_};
        
        joint_pub_->publish(js);
    }

    // Parameters
    std::string port_name_;
    int baudrate_;
    std::vector<int64_t> dxl_ids_;
    double wheel_radius_;
    double wheel_base_;
    int pulses_per_rot_;
    double control_rate_;
    double low_battery_threshold_;
    bool publish_tf_;

    double circumference_;
    double rpm_to_value_scale_;
    double rpm_to_rads_;

    // Dynamixel SDK
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWrite_;
    std::unique_ptr<dynamixel::GroupBulkRead> groupBulkRead_;

    // ROS Interfaces
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    int32_t last_encoder_l_ = 0;
    int32_t last_encoder_r_ = 0;
    double last_rpm_l_ = 0.0;
    double last_rpm_r_ = 0.0;
    rclcpp::Time last_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<PinkyBringup>());
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("pinky_bringup_cpp"), "Exception in node: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
