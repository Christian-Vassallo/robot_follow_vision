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
    ros::Publisher publisherhead = node.advertise<std_msgs::Float64MultiArray>("HeadHRP2", 2000);
    ros::Publisher publisherchest = node.advertise<std_msgs::Float64MultiArray>("ChestHRP2", 2000);
    ros::Publisher publisherterrain = node.advertise<std_msgs::Float64MultiArray>("Box", 2000);

    MoCapMessenger headrobot;
    MoCapMessenger chestrobot;
    MoCapMessenger terrainbox;

    headrobot.sub = headrobot.n.subscribe("/evart/hrp2_head_sf/PO", 2000, &MoCapMessenger::callbackFunction, &headrobot);
    chestrobot.sub = chestrobot.n.subscribe("/evart/HRP2_waist_sensor_frame/PO", 2000, &MoCapMessenger::callbackFunction, &chestrobot);
    terrainbox.sub = terrainbox.n.subscribe("/evart/carpet/PO", 2000, &MoCapMessenger::callbackFunction, &terrainbox);



    std::vector<double> headrobot_data_t;
    std::vector<std::vector<double> > headrobot_data;
    std::vector<double> chestrobot_data_t;
    std::vector<std::vector<double> > chestrobot_data;
    std::vector<double> terrainbox_data_t;
    std::vector<std::vector<double> > terrainbox_data;

    bool stop_flag = false;

    // Initialization MoCap
    std::cout << "Bringing up.." << std::flush;
    double timenow = time(NULL)+1;
    while(time(NULL)<timenow){
        ros::spinOnce();
        ros::Duration(0.02).sleep();
    }


    // Starting to record motion if the object is visible
    std::cout << "\nStart" << std::endl;
    timenow = time(NULL)+40;
    while(ros::ok() && time(NULL)<timenow){
        ros::spinOnce();

        /// Take data from MoCap, tracking the Actor
        headrobot_data_t.clear();
        headrobot_data_t = headrobot.item_XY_YAW_configuration();
        headrobot_data.push_back(headrobot_data_t);

        chestrobot_data_t.clear();
        chestrobot_data_t = chestrobot.item_XY_YAW_configuration();
        chestrobot_data.push_back(chestrobot_data_t);

        terrainbox_data_t.clear();
        terrainbox_data_t = terrainbox.item_XY_YAW_configuration();
        terrainbox_data.push_back(terrainbox_data_t);

        std::cout << "Object: x " << headrobot_data_t[0] << " y " << headrobot_data_t[1] << "th " << headrobot_data_t[2] << std::endl;
        ros::Duration(0.01).sleep();
    }

    std::cout << "Sending Data to Matlab" << std::endl;
    Eigen::MatrixXd headrobot_data_matrix = Eigen::MatrixXd::Zero(headrobot_data.size(),3);
    Eigen::MatrixXd chestrobot_data_matrix = Eigen::MatrixXd::Zero(chestrobot_data.size(),3);
    Eigen::MatrixXd terrainbox_data_matrix = Eigen::MatrixXd::Zero(terrainbox_data.size(),3);

    std_msgs::Float64MultiArray headrobot_data_msg;
    std_msgs::Float64MultiArray chestrobot_data_msg;
    std_msgs::Float64MultiArray terrainbox_data_msg;

    for(int c=0; c<headrobot_data.size(); ++c){
       headrobot_data_matrix(c,0) = headrobot_data[c][0]; headrobot_data_matrix(c,1) = headrobot_data[c][1]; headrobot_data_matrix(c,2) = headrobot_data[c][2];
       chestrobot_data_matrix(c,0) = chestrobot_data[c][0]; chestrobot_data_matrix(c,1) = chestrobot_data[c][1]; chestrobot_data_matrix(c,2) = chestrobot_data[c][2];
       terrainbox_data_matrix(c,0) = terrainbox_data[c][0]; terrainbox_data_matrix(c,1) = terrainbox_data[c][1]; terrainbox_data_matrix(c,2) = terrainbox_data[c][2];
      }

    // Define the message to send
    tf::matrixEigenToMsg(headrobot_data_matrix,headrobot_data_msg);
    tf::matrixEigenToMsg(chestrobot_data_matrix,chestrobot_data_msg);
    tf::matrixEigenToMsg(terrainbox_data_matrix,terrainbox_data_msg);

    for(unsigned int i=0; i<1000; i++){
      publisherhead.publish(headrobot_data_msg);
      publisherchest.publish(chestrobot_data_msg);
      publisherterrain.publish(terrainbox_data_msg);
      }

    std::cout << "Done. " << std::endl;


    return 0;
}

