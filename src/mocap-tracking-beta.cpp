/// Tracking Robulab with MoCap

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64MultiArray.h>

#include <sstream>

#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/Sound.h>
#include <nav_msgs/Odometry.h>

#include "mocapmessenger.hpp"
#include "Robulab10Class.hpp"



int main(int argc, char **argv)
{

    ros::init(argc, argv, "mocaptrackingrobot");

    ros::NodeHandle node;
    ros::Publisher publisher = node.advertise<std_msgs::Float64MultiArray>("ObjDataMocap", 2000);

    MoCapMessenger mocap;
    mocap.sub = mocap.n.subscribe("/evart/robulabhalf/PO", 2000, &MoCapMessenger::callbackFunction, &mocap);

    Robulab10 Robot;
    Robot.establish_connection();

    std::vector<double> item_data_t;
    std::vector<std::vector<double> > item_data;

    bool stop_flag = false;

    // Initialization MoCap
    std::cout << "Bringing up.." << std::flush;
    double timenow = time(NULL)+0.1;
    while(time(NULL)<timenow){
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }


    // Starting to record motion if the object is visible
    std::cout << "\nStart" << std::endl;
    while(ros::ok() && !stop_flag){
        ros::spinOnce();

        /// Take data from MoCap, tracking the Actor
        item_data_t.clear();
        item_data_t = mocap.item_XY_YAW_configuration_OFFSET(-2.23393);
        item_data.push_back(item_data_t);

        std::cout << "Object: x " << item_data_t[0] << " y " << item_data_t[1] << "th " << item_data_t[2] << std::endl;
        ros::Duration(0.005).sleep();
    }

    std::cout << "Sending Data to Matlab" << std::endl;
    Eigen::MatrixXd item_data_matrix = Eigen::MatrixXd::Zero(item_data.size(),3);
    std_msgs::Float64MultiArray item_data_msg;


    for(int c=0; c<item_data.size(); ++c){
       item_data_matrix(c,0) = item_data[c][0]; item_data_matrix(c,1) = item_data[c][1]; item_data_matrix(c,2) = item_data[c][2];
    }

    // Define the message to send
    tf::matrixEigenToMsg(item_data_matrix,item_data_msg);

    std::cout << "Done. " << std::endl;
    publisher.publish(item_data_msg);

    return 0;
}

