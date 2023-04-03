#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Float64.h"
#include "actionlib/server/simple_action_server.h"
#include "twist_control_msgs/IndicatorAction.h"
#include "read_sensor/tactile_sensor_data.h"
#include <yaml-cpp/yaml.h>
#include "controller_manager_msgs/SwitchController.h"

bool stop = false;
double xc, yc, Ix, Iy, I;
double xc0 = 0.0;
double yc0 = 0.0;
double spatial_resolution = 3.55;
double p = 1.775;
std::array<double, 12> xi = {-p, p, -p, p, -p, p, -p, p, -p, p, -p, p};
std::array<double, 12> yi = {5 * p, 5 * p, 3 * p, 3 * p, p, p, -p, -p, -3 * p, -3 * p, -5 * p, -5 * p};
std::vector<float> data_read;

void sensor_data_cb(const read_sensor::tactile_sensor_data::ConstPtr &msg)
{
    data_read = msg->tactile_sensor_data;
}

class IndicatorActionServer
{
public:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<twist_control_msgs::IndicatorAction> as_;
    twist_control_msgs::IndicatorResult result;

    IndicatorActionServer()
        : as_(nh_, "indicator", boost::bind(&IndicatorActionServer::executeCB, this, _1), false)
    {
        as_.start();
    }

    void compute_indicator()
    {
        double den = 0.0;
        double numx = 0.0;
        double numy = 0.0;
        for (int i = 0; i < 12; i++)
        {
            den = den + data_read.at(i);
            numx = numx + data_read.at(i) * xi[i];
            numy = numy + data_read.at(i) * yi[i];
        }
        xc = numx / den;
        yc = numy / den;
        Ix = xc - xc0;
        Iy = yc - yc0;
        I = std::sqrt(Ix * Ix + Iy * Iy);
    }

    void zero_indicator()
    {
        compute_indicator();
        xc0 = xc;
        yc0 = yc;
    }

    double compute_velocity(double t, double tc, double vc)
    {
        if (t <= 0.0)
            return 0.0;
        if (t >= tc)
            return vc;
        if (t < tc)
            return (vc / tc) * t;
            
    }

    double stop_robot(double t, double ta, double v_arresto)
    {
        if (t >= ta)
            return 0.0;
        if (t <= 0)
            return v_arresto;
        if (t > 0 && t < ta)
            return -(v_arresto / ta) * (t - ta);
    }

    void executeCB(const twist_control_msgs::IndicatorGoalConstPtr &goal)
    {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        bool success = true;

        // switch controllers
        ros::ServiceClient switch_controller = nh_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");

        std::vector<std::string> start_controller;
        start_controller.push_back("twist_controller");
        std::vector<std::string> stop_controller;
        stop_controller.push_back("scaled_pos_joint_traj_controller");
        controller_manager_msgs::SwitchController switch_controller_req;
        switch_controller_req.request.start_controllers = start_controller;
        switch_controller_req.request.stop_controllers = stop_controller;
        switch_controller_req.request.strictness = 1;
        switch_controller_req.request.start_asap = false;
        ros::service::waitForService("/controller_manager/switch_controller", ros::Duration(1));
        switch_controller.call(switch_controller_req);
        if (switch_controller_req.response.ok)
        {
            ROS_INFO_STREAM("Controller switch correctly");
        }
        else
        {
            ROS_ERROR_STREAM("Error occured trying to switch controller");
            success = false;
            as_.setAborted();
            return;
        }

        ros::Publisher twist_pub = nh_.advertise<geometry_msgs::Twist>("/twist/cmd", 1);
        ros::Publisher indicator_pub = nh_.advertise<std_msgs::Float64>("/indicator_data", 1);
        ros::Publisher distance_pub = nh_.advertise<std_msgs::Float64>("/distance_data", 1);
        zero_indicator();

        geometry_msgs::Twist twist;
        std_msgs::Float64 indicator;
        std_msgs::Float64 distance;

        double dist = 0.0;
        double vel = 0.0;
        double rate = 100;
        ros::Time t0 = ros::Time::now();
        ros::Duration t = ros::Time::now() - t0;
        ros::Rate loop_rate(rate);

        stop = false;

        while (ros::ok() && !stop)
        {
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("Preempt");
                as_.setPreempted();
                success = false;
                break;
            }

            compute_indicator();

            geometry_msgs::TransformStamped base_to_fingerpad;
            while (!tfBuffer.canTransform("fingerpad", "base", ros::Time(0)))
            {
                ;
            }
            try
            {
                base_to_fingerpad = tfBuffer.lookupTransform("fingerpad", "base",
                                                             ros::Time(0));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }

            tf2::Quaternion b_Q_t;
            tf2::fromMsg(base_to_fingerpad.transform.rotation, b_Q_t);
            tf2::Matrix3x3 b_R_t(b_Q_t);

            if (I > goal->indicator_threshold && dist >= goal->distance)
            {
                ROS_INFO("Threshold reached");
                indicator.data = I;
                indicator_pub.publish(indicator);
                
                double last_velocity = twist.linear.x;

                dist = dist + (1/rate)*abs(last_velocity);
                distance.data = dist;
                distance_pub.publish(distance);
                
                ros::Time t0_s = ros::Time::now();
                ros::Duration t_s = ros::Time::now() - t0_s;
                while (ros::ok() && t_s.toSec() <= goal->t_stop)
                {
                    tf2::Vector3 v_t(stop_robot(t_s.toSec(), goal->t_stop, last_velocity),
                                     0.0,
                                     0.0);
                    v_t = b_R_t * v_t;
                    twist.linear.x = v_t[0]; // stop_robot(t_s.toSec(), goal->t_stop, last_velocity);
                    twist.linear.y = v_t[1];
                    twist.linear.z = v_t[2];
                    twist.angular.x = 0.0;
                    twist.angular.y = 0.0;
                    twist.angular.z = 0.0;

                    twist_pub.publish(twist);

                    t_s = ros::Time::now() - t0_s;
                    loop_rate.sleep();

                    dist = dist + (1/rate)*abs(v_t[0]);
                    distance.data = dist;
                    distance_pub.publish(distance);

                    compute_indicator();
                    indicator.data = I;
                    indicator_pub.publish(indicator);
                }

                stop = true;
                break;
            }

            indicator.data = I;
            indicator_pub.publish(indicator);
            vel = compute_velocity(t.toSec(), goal->tc, goal->vel_max);
            tf2::Vector3 v_t(vel,
                             0.0,
                             0.0);
            v_t = b_R_t * v_t;

            dist = dist + (1/rate)*abs(vel);
            distance.data = dist;
            distance_pub.publish(distance);

            twist.linear.x = v_t[0]; // compute_velocity(t.toSec(), goal->tc, goal->vel_max);
            twist.linear.y = v_t[1];
            twist.linear.z = v_t[2];
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = 0.0;

            twist_pub.publish(twist);

            t = ros::Time::now() - t0;
            loop_rate.sleep();
        }

        // switch controllers
        switch_controller_req.request.start_controllers = stop_controller;
        switch_controller_req.request.stop_controllers = start_controller;
        switch_controller_req.request.strictness = 1;
        switch_controller_req.request.start_asap = false;
        ros::service::waitForService("/controller_manager/switch_controller", ros::Duration(1));
        switch_controller.call(switch_controller_req);
        if (switch_controller_req.response.ok)
        {
            ROS_INFO_STREAM("Controller switch correctly");
        }
        else
        {
            ROS_ERROR_STREAM("Error occured trying to switch controller");
            success = false;
            as_.setAborted();
            return;
        }
        //

        if (success)
        {
            result.success = true;
            ROS_INFO("Succeeded");
            as_.setSucceeded(result);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "indicator");

    // ros::NodeHandle nh("~");
    // std::string serialport;
    // std::string sensor_name;
    // nh.getParam("serialport",serialport);
    // nh.getParam("sensor_name",sensor_name);
    std::string pkg_path = ros::package::getPath("twist_control");
    YAML::Node node = YAML::LoadFile(pkg_path + "/cfg/sensors_features.yaml");
    std::string serialport = node["sensors"]["tactile"]["serialport"].as<std::string>();
    std::string sensor_name = node["sensors"]["tactile"]["name"].as<std::string>();
    ros::NodeHandle nh_;

    std::string topicName = "TactileData_dev_" + serialport + "_" + sensor_name;
    ros::Subscriber ind_sub = nh_.subscribe(topicName, 1, sensor_data_cb);
    ROS_INFO("Subscribe to tactile data");

    IndicatorActionServer ind;

    ros::spin();

    return 0;
}