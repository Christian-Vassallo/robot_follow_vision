
#include "ros/ros.h"
#include <sstream>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <stdlib.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <time.h>
#include <string.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "std_msgs/String.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

#include "Robulab10Class.hpp"

double CirclesData[2][3];
static bool stop_interrupt = false;

void getDataCircles(const std_msgs::Float64MultiArray::ConstPtr& array);
void my_handler(int s);

void getDataCircles(const std_msgs::Float64MultiArray::ConstPtr& array)
{
  CirclesData[0][0] = array->data[0];
  CirclesData[0][1] = array->data[1];
  CirclesData[0][2] = array->data[2];
  CirclesData[1][0] = array->data[3];
  CirclesData[1][1] = array->data[4];
  CirclesData[1][2] = array->data[5];

  return;
}

void my_handler(int s){
           Robulab10 Robot;
           printf("Caught signal %d\n",s);
           stop_interrupt = true;

           Robot.establish_connection();
           Robot.move_robot(0,0);
}

int main(int argc, char* argv[])
{
   ros::init(argc, argv, "robotcontrol");
   ros::start();

   // Init ROS
   ros::NodeHandle node_message;
   ros::Subscriber _subscriber = node_message.subscribe("datacircles", 2000, &getDataCircles);

   // Init Robulab and Mocap class
   Robulab10 Robot;

   std::vector<double> Robot_configuration, Goal_Configuration;
   // Open the connection with the Robot Robulab
   Robot.establish_connection();

   // Active signal (CTRL + C interrupt)
   struct sigaction sigIntHandler;
   sigIntHandler.sa_handler = my_handler;
   sigemptyset(&sigIntHandler.sa_mask);
   sigIntHandler.sa_flags = 0;
   sigaction(SIGINT, &sigIntHandler, NULL);

   /// Elliptical Coordinates Control Feedback

   double e1, e2, e3, theta_tilda;
   double x1, y1, x2, y2, A, B, C, xi, eta, betae;
   double x1centered, y1centered, x2centered, y2centered;
   double lambda, omega, omegao, K1, K2;
   double k1, k2, k3, k4;
   double u1, u2, v, w;
   bool goal_reached = false;
   unsigned int iter=0;

   u1 = 0;   u2 = 0;
   v = 0;    w = 0;


   /// Parameter of the camera (defined by calibration)
   double alphax =  712.27744;
   double alphay =  711.92580;
   double PPx = 330.98367;        // Principal point X
   double PPy = 273.41515;        // Principal point Y
   double a = 31.5/2;

   /// Delay before to get information from MoCap (to avoid null data)
   std::cout << "... Initilization Virtual Tracking..." << std::endl;

   double timenowdouble = time(NULL)+0.15;
   while(time(NULL)<timenowdouble){
       ros::spinOnce();
       ros::Duration(0.01).sleep();
     }

   /// TRACK CONTROL
   while(ros::ok() && goal_reached == false && stop_interrupt==false ){

       ros::spinOnce();

       /// Gains
       k1 = 5;
       k2 = 5;
       k3 = 1;
       k4 = 1;

       /// Check if the GOAL is reached --> When error < eps -> stop : fix it
       if (false){
           std::cout << "-- GOAL REACHED AND ROBOT STOPPED -- " << std::endl;
           Robot.move_robot(0,0); // Stop the Robot (if Robulab)
           goal_reached = true;
       }

       /// Elliptic coordinates computed from the image plane measurements
       /// Input parser...(feature points)

       x1centered = CirclesData[0][0] - PPx;
       y1centered = CirclesData[0][1] - PPy;
       x2centered = CirclesData[1][0] - PPx;
       y2centered = CirclesData[1][1] - PPy;

       x1 = x1centered;
       y1 = (y1centered/fabs(y1centered))*y1centered;
       x2 = x2centered;
       y2 = (y2centered/fabs(y2centered))*y2centered;

       std::cout << " ---------------------------------- " << std::endl;
       std::cout << " x1 = " << x1 << " y1 = " << y1 << std::endl;
       std::cout << " x2 = " << x2 << " y2 = " << y2 << std::endl;
       std::cout << " ---------------------------------- " << std::endl;

       A = y2*sqrt(x1*x1+alphax*alphax);
       B = y1*sqrt(x2*x2+alphax*alphax);
       C = sqrt((x1*y2-x2*y1)*(x1*y2-x2*y1)+alphax*alphax*(y2-y1)*(y2-y1));

       /// Elliptic coordinates
       xi = acosh((A+B)/C);
       eta = M_PI/2-acos((A-B)/C);
       betae = -(atan(x1/alphax)+(atan(x2/alphax)-atan(x1/alphax))/2);

       /// Control law in elliptic coordinates (Path Following like)
       K1 = 0.1;       // Gain for the forward velocity
       K2 = 5.5/10;   // How fast beta converges to zero
       lambda = 6;    // How fast eta converges to zero
       v = a * K1 * cos(eta) * cosh(xi) * sqrt(1+tan(eta) * tan(eta) * tanh(xi) * tanh(xi));
       if (betae<=1e-2)
           omegao = -lambda*eta*K1-K2*betae;
       else
           omegao = -lambda*eta*K1*sin(betae)/betae-K2*betae;

       omega = -omegao+K1*pow((cos(2*eta)+cosh(2*xi)),-1)*((-2)*cos(betae)*cos(eta)*sin(eta)+sin(betae)*sinh(2*xi));

       w = omega;

        // Some print out
       std::cout << " ----------------------------------------- " << std::endl;
       std::cout << "beta: " << eta << std::endl;
       std::cout << "eta: " << betae << std::endl;

       std::cout << "Robot controls - v: " << v << " omega: " << w <<  " omegao: " << omegao << std::endl;
       //std::cout << " ----------------------------------------- " << std::endl;

       /// Move ROBOT
       //Robot.move_robot(v, w);


       ros::Duration(0.01).sleep();

       ++iter;
   }
   std::cout << "out" << std::endl;

   Robot.move_robot(0, 0);

  // for(unsigned int pp=0; pp<vitesserobot.size();pp++)
   //  std::cout << vitesserobot[pp] << std::endl;

   return 0;

}
