
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

void remove_distorsion(double *pixelxy, double *cc, double *fc, double *kc, double alpha_c, double *xp)
{
  double x, y;
  double r;
  double dx, xd;
  double dy, yd;

  x = pixelxy[0]/fc[0];
  y = pixelxy[1]/fc[1];

  r = sqrt(x*x+y*y);

  dx = 2*kc[2]*x*y+kc[3]*(r*r+2*x*x);
  xd = (1+kc[0]*r*r+kc[1]*pow(x,4)+kc[4]*pow(x,6))*x+dx;
  dy = kc[2]*(r*r+2*y*y)+2*kc[3]*x*y;
  yd = (1+kc[0]*r*r+kc[1]*pow(r,4)+kc[4]*pow(r,6))*y+dy;

  xp[0] = xd*fc[0]  ;
  xp[1] = yd*fc[1] ;
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
   double a = 72/2;
   double kc[5] = {-0.39852, 0.24318, 0.00180, -0.00011, 0.00000};
   double pixelxy[2];
   double cc[2] = {PPx, PPy};
   double fc[2] = {alphax, alphay};
   double alpha_c = 0;
   double xp[2];
   double x1p, y1p;

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


       /// Check if the GOAL is reached --> When error < eps -> stop : fix it
       if (false){
           std::cout << "-- GOAL REACHED AND ROBOT STOPPED -- " << std::endl;
           Robot.move_robot(0,0); // Stop the Robot (if Robulab)
           goal_reached = true;
       }

       /// Elliptic coordinates computed from the image plane measurements
       /// Input parser...(feature points)

       if(CirclesData[0][0]>CirclesData[1][0])
         {
           x1centered = CirclesData[1][0] - PPx;
           y1centered = CirclesData[1][1] - PPy;
           x2centered = CirclesData[0][0] - PPx;
           y2centered = CirclesData[0][1] - PPy;
         }
       else{
           x1centered = CirclesData[0][0] - PPx;
           y1centered = CirclesData[0][1] - PPy;
           x2centered = CirclesData[1][0] - PPx;
           y2centered = CirclesData[1][1] - PPy;
         }

       std::cout << " Prima x1 translated" << x1centered << std::endl;
       std::cout << " Prima y1 translated" << y1centered << std::endl;
       std::cout << " Prima x2 translated" << x2centered << std::endl;
       std::cout << " Prima y2 translated" << y2centered << std::endl;
/*
       pixelxy[0] = x1centered;
       pixelxy[1] = y1centered;
       remove_distorsion(pixelxy, cc, fc, kc, alpha_c, xp);
       x1 = xp[0];
       y1 =( xp[1]/fabs(xp[1]))*xp[1];

*/
      x1 = x1centered;
      y1 = (y1centered/fabs(y1centered))*y1centered;

/*
      pixelxy[0] = x2centered;
      pixelxy[1] = y2centered;
      remove_distorsion(pixelxy, cc, fc, kc, alpha_c, xp);
      x2 = xp[0];
      y2 =( xp[1]/fabs(xp[1]))*xp[1];
*/
       x2 = x2centered;
       y2 = (y2centered/fabs(y2centered))*y2centered;

       /*
       std::cout << " ---------------------------------- " << std::endl;
       std::cout << " x1 = " << x1 << " y1 = " << y1 << std::endl;
       std::cout << " x2 = " << x2 << " y2 = " << y2 << std::endl;
       std::cout << " ---------------------------------- " << std::endl;
*/



       std::cout << " ---------------------------------- " << std::endl;
       std::cout << " ellipse1x = " << CirclesData[0][0] << " normalized " << x1 << std::endl;
       std::cout << " ellipse1y = " << CirclesData[0][1] << " normalized " << y1 << std::endl;
       std::cout << " ellipse2x = " << CirclesData[1][0] << " normalized " << x2 << std::endl;
       std::cout << " ellipse2y = " << CirclesData[1][1] << " normalized " << y2 << std::endl;
       std::cout << " ---------------------------------- " << std::endl;

       A = y2*sqrt(x1*x1+alphax*alphax);
       B = y1*sqrt(x2*x2+alphax*alphax);
       C = sqrt((x1*y2-x2*y1)*(x1*y2-x2*y1)+alphax*alphax*(y2-y1)*(y2-y1));

       std::cout << " A-B " << A-B << " A+B " << A+B << " C " << C << std::endl;
       double P = (A+B)/C;
       double Pm = (A-B)/C;
       /// Elliptic coordinates
       xi = acosh(P);
       eta = M_PI/2-acos(Pm);
       betae = -(atan(x1/alphax)+(atan(x2/alphax)-atan(x1/alphax))/2);

       /// Control law in elliptic coordinates (Path Following like)
       K1 = 0.001;       // Gain for the forward velocity
       K2 = 5.5/10;   // How fast beta converges to zero
       lambda = 1000;    // How fast eta converges to zero
       v = a * K1 * cos(eta) * cosh(xi) * sqrt(1+tan(eta) * tan(eta) * tanh(xi) * tanh(xi));
       if (betae<=0.01)
           omegao = -lambda*eta*K1-K2*betae;
       else
           omegao = -lambda*eta*K1*sin(betae)/betae-K2*betae;

       std::cout << "sinb/b " << sin(betae)<<"/"<<betae << " = " << sin(betae)/betae << std::endl;
       omega = -omegao+K1*(1/(cos(2*eta)+cosh(2*xi)))*((-2)*cos(betae)*cos(eta)*sin(eta)+sin(betae)*sinh(2*xi));

       w = omega;

        // Some print out
       std::cout << " ----------------------------------------- " << std::endl;
       std::cout << "xi " << xi << std::endl;
       std::cout << "eta: " << eta << std::endl;
       std::cout << "betae: " << betae << std::endl;
       std::cout << "Robot controls - v: " << v << " omega: " << w <<  " omegao: " << omegao << std::endl;
       //std::cout << " ----------------------------------------- " << std::endl;

       /// Move ROBOT
       if(CirclesData[1][1]<20 || CirclesData[0][1]<20){
         Robot.move_robot(0, 0);
         stop_interrupt=true;
         }
       else
          Robot.move_robot(v, w);


       ros::Duration(0.02).sleep();

       ++iter;
   }
   std::cout << "out" << std::endl;

   //Robot.move_robot(0, w);

  // for(unsigned int pp=0; pp<vitesserobot.size();pp++)
   //  std::cout << vitesserobot[pp] << std::endl;



   return 0;

}
