#include "mas/mas_node.hpp"

Controller::Controller() : tfListener(tfBuffer)
{
    getParamOrError("id", id);
    getParamOrError("/num_robots", num_robots);
    getParamOrError("/ns_prefix", ns_prefix);
    getParamOrError("/gain", gain);
    getParamOrError("/gain_yaw", gain_yaw);
    getParamOrError("/threshold_yaw", threshold_yaw);

    ROS_INFO("id: %d, %d in total", id, num_robots);

    std::stringstream log;
    log << "Neighbors: ";

    pub = nh.advertise<Twist>("cmd_vel", 10);

    for (int id_ = 0; id_ < num_robots; id_++)
    {
        // assume prefix is tb3_ID
        if (id_ != id)
        {
            id_neighbors.push_back(id_);
            log << std::to_string(id_) + " ";
        }
        robots_initialized.push_back(false);
        transforms.push_back(tf2::Transform());
        auto topic_name = ns_prefix + std::to_string(id_) + "/odom";
        auto callback = boost::bind(&Controller::odomCallBack, this, _1, id_);
        subs_odom.push_back(nh.subscribe<Odometry>(topic_name, 100, callback));
    }
    ROS_INFO("%s", log.str().c_str());

    while (!are_all_robots_initialized())
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
}

void Controller::moveForSecond(const double linear_vel, const double s)
{
    ros::Time begin = ros::Time::now();
    int step = 0;
    ros::Duration diff(0, 0); // diff.sec = diff.nsec = 0と同じ
    setVel(linear_vel);       //  指定速度で進む

    ros::Rate rate(10); // ループの頻度を設定
    rate.sleep();

    while (diff < ros::Duration(s))
    {
        ros::spinOnce();
        if (step++ == 0)
            begin = ros::Time::now();

        diff = ros::Time::now() - begin;
        ROS_INFO("ROS diff: %u.%u", diff.sec, diff.nsec);
        setVel(linear_vel); //  指定速度で進む
        rate.sleep();
    }
    setVel(0); // 停止
}

void Controller::consensusControl()
{
    auto pos = transforms[id].getOrigin();
    auto diff_sum_world = tf2::Vector3(0, 0, 0); // in world frame
    for (auto id_neighbor : id_neighbors)
    {
        diff_sum_world += transforms[id_neighbor].getOrigin() - pos;
    }
    diff_sum_world *= gain;
    ROS_INFO("diff_world x:%f, y:%f, z:%f", diff_sum_world.x(), diff_sum_world.y(), diff_sum_world.z());
    // ROS_INFO("pos x:%f, y:%f, z:%f", pos.x(), pos.y(), pos.z());
    // ROS_INFO("pos0 x:%f, y:%f, z:%f", transforms[1].getOrigin().x(), transforms[1].getOrigin().y(), transforms[1].getOrigin().z());

    // 現在時刻にした方がよいか。。？
    auto transformStamped = tfBuffer.lookupTransform(ros::this_node::getNamespace().substr(1) + "/base_link", "world", ros::Time(0), ros::Duration(1.0));
    tf2::Transform t;
    fromMsg(transformStamped.transform, t);
    t.setOrigin(tf2::Vector3(0, 0, 0)); //回転のみ考慮
    // ROS_INFO("mat x:%f, y:%f, z:%f", t.getOrigin()[0], t.getOrigin()[1], t.getOrigin()[2]);
    tf2::Vector3 diff_body = t * diff_sum_world; // world to body
    diff_body[2] = 0;
    ROS_INFO("diff_body x: %f, y:%f, z:%f", diff_body.x(), diff_body.y(), diff_body.z());
    auto [x, yaw] = steerlingLaw(diff_body);
    setVel(std::clamp(x, -0.25, 0.25), std::clamp(yaw, -0.08726646259, 0.08726646259));
    ROS_INFO("x: %f, yaw: %f", x, yaw);
}

std::tuple<double, double> Controller::steerlingLaw(tf2::Vector3 vec_in)
{
    // tf2::Vector3 vec_x = tf2::Vector3(1, 0, 0);
    auto theta = atan2(vec_in[1], vec_in[0]);
    ROS_INFO("theta: %f", theta);

    auto x = vec_in.x();
    auto yaw = 0.0;
    auto theta_abs = abs(theta);
    if (theta_abs <= M_PI_2)
    {
        yaw = gain_yaw * theta;
        if (theta_abs >= threshold_yaw)
        {
            x = 0;
        }
        else
        {
            x *= cos(theta);
        }
    }
    else if (theta > 0)
    {
        double delta = theta - M_PI;
        yaw = gain_yaw * delta;
        if (delta >= threshold_yaw)
        {
            x = 0;
        }
        else
        {
            x *= cos(delta);
        }
    }
    else
    {
        double delta = theta + M_PI;
        yaw = gain_yaw * delta;
        if (delta >= threshold_yaw)
        {
            x = 0;
        }
        else
        {
            x *= cos(delta);
        }
    }
    return std::tuple(x, yaw);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mas_node");
    NodeHandle nh;
    // int move_seconds = 0;
    // double move_velocity = 0.0;
    // nh.getParam("/move_seconds", move_seconds);
    // nh.getParam("/move_velocity", move_velocity);
    // ROS_INFO("seconds:%d, vel: %.1f", move_seconds, move_velocity);
    Controller c;

    // c.moveForSecond(move_velocity, move_seconds);
    ros::Rate rate(50); // ループの頻度を設定
    while (ros::ok())
    {
        c.consensusControl();
        ros::spinOnce();
        rate.sleep();
    }
    c.stop();
    return 0;
}
