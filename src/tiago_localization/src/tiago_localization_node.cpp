#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tiago_localization/localizer.h>

namespace tiago_localization
{

    Localizer::Localizer()
    {
    }

    Localizer::~Localizer()
    {
    }

    bool Localizer::initialize(ros::NodeHandle& nh)
    {
        covariance_sum = 0.0;
        // setup ros connections

        //#>>>>TODO: Subscribe to the pose msg published by the package amcl: amcl_pose                
        pose_sub_ = nh.subscribe("/amcl_pose", 1, &Localizer::poseCallback, this);
        //#>>>> Hint: see http://wiki.ros.org/amcl

        //#>>>>TODO: advertise a geometry_msgs::Twist to command the robot by velocity
        vel_pub_ = nh.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);

        //#>>>>TODO: create a serviceClient that calls /global_localization service with empty srv of type std_srvs::Empty
        localization_client_ = nh.serviceClient<std_srvs::Empty>("/global_localization");


        if(!ros::param::get("/spin_speed"/*#>>>>TODO: PARAMETER NAME*/, speed))
            return false;
        if(!ros::param::get("/threshold"/*#>>>>TODO: PARAMETER NAME*/, threshold))
            return false;
        
        
        //#>>>>TODO: create an empty srv
        std_srvs::Empty empty_srv;

        if (localization_client_.call(empty_srv/*#>>>>TODO: YOUR SRV NAME*/)) {
            ROS_INFO("Global localization initialized.");
        } else {
            ROS_ERROR("Failed to call service /global_localization");
        }
        
        return true;

    }

    void Localizer::robot_spin(double speed) {
            
            geometry_msgs::Twist msg;
            msg.angular.z = speed;  // adjust value as needed for the robot to spin

            ros::Rate rate(10);  // 10 Hz
            while (ros::ok()) {
                
                vel_pub_.publish(msg);
                ros::spinOnce();
                rate.sleep();
            }
        }
    //----------------------------------------------------------------------------
    // callbacks


    void Localizer::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        //#>>>>TODO: Calculate the time interval between the current ROS time and the timestamp in the message.
        double time_interval = (ros::Time::now() - msg->header.stamp).toSec();
        covariance_sum = 0;
        if (time_interval <= 0.5/*#>>>>TODO: CHECK THE INTERVAL IS WITHIN 0.5 SECOND*/){
            //#>>>>TODO: Calculate the sum of the covariance vector in the msg (covariance_sum)
            for (int i = 0; i < 36; ++i)
            {
                covariance_sum += msg->pose.covariance[i];
            }
            
            if (covariance_sum < threshold) {
                ROS_INFO("Localization successful, covariance sum below 0.1. Terminating node.");
                ros::shutdown();
            }
        }
        

    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tiago_localization_node");
    tiago_localization::Localizer localizer;

    ros::NodeHandle nh;
    if(!localizer.initialize(nh))
    {
        ROS_ERROR_STREAM("Tiago_localizer::Localizer failed to initialize");
        return -1;
    }
    ROS_INFO("Publishing robot vel cmd.");
    localizer.robot_spin(localizer.speed);
    ros::spin();
    return 0;
}
