#pragma once

#include <tuple>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>

using geometry_msgs::Pose;
using geometry_msgs::Twist;
using nav_msgs::Odometry;
using ros::NodeHandle;
using ros::Publisher;
using ros::Subscriber;
using std::string;
using std::vector;

class Controller
{
private:
    int id;
    int num_robots;
    string ns_prefix; // something like /tb3_ (absolute path)
    vector<int> id_neighbors;
    vector<bool> robots_initialized;
    Twist vel;
    vector<tf2::Transform> transforms;
    NodeHandle nh;
    Publisher pub;
    vector<Subscriber> subs_odom;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    double gain, gain_yaw, threshold_yaw;

public:
    Controller();
    void setLinearVel(const double linear_vel);   // 並進速度の設定
    void setAngularVel(const double angular_vel); // 回転速度の設定
    void setVel(const double linear_vel, const double angular_vel);
    void setLinear(const double linear_vel, const double angular_vel);
    void odomCallBack(const Odometry::ConstPtr &msg, const int id);
    void moveForSecond(const double linear_vel, const double s); //　指定速度と時間で移動
    void consensusControl();
    std::tuple<double, double> steerlingLaw(tf2::Vector3 vec_in);
    bool are_all_robots_initialized();
    void stop()
    {
        setVel(0, 0);
    }
    template <typename T>
    void getParamOrError(string key, T &val)
    {
        if (!nh.getParam(key, val))
            throw std::runtime_error("ROS parameter not found: " + key);
    }
};

// 並進速度の設定
inline void Controller::setLinearVel(const double linear_vel)
{
    vel.linear.x = linear_vel;
    pub.publish(vel);
}

// 回転速度の設定
inline void Controller::setAngularVel(const double angular_vel)
{
    vel.angular.z = angular_vel;
    pub.publish(vel);
}

inline void Controller::setVel(const double linear_vel, const double angular_vel = 0)
{
    vel.linear.x = linear_vel;
    vel.angular.z = angular_vel;
    pub.publish(vel);
}

inline void Controller::odomCallBack(const Odometry::ConstPtr &msg, const int id_)
{
    try
    {
        auto transformStamped = tfBuffer.lookupTransform("world", ns_prefix.substr(1) + std::to_string(id_) + "/odom", msg->header.stamp, ros::Duration(1.0));
        auto pose_world = Pose();
        tf2::doTransform(msg->pose.pose, pose_world, transformStamped);
        tf2::convert(pose_world, transforms[id_]);
        robots_initialized[id_] = true;
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform turtle2 to turtle1: %s", ex.what());
    }
}

inline bool Controller::are_all_robots_initialized()
{
    return std::all_of(robots_initialized.begin(), robots_initialized.end(), [](bool robotinitialized) { return robotinitialized; });
}
