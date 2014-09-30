
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
#include <eigen_conversions/eigen_msg.h>

#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"

#include "Robulab10Class.hpp"
#include "mocapmessenger.hpp"

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

// Function to Remove distorsion
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



   // Init Robulab and MocapRobulab class
   Robulab10 Robot;
   MoCapMessenger MocapRobulab;
   MoCapMessenger MocapGoalLeft;
   MoCapMessenger MocapGoalRight;

   // To receive data from MocapRobulabRobulab
   MocapRobulab.sub = MocapRobulab.n.subscribe("/evart/robulabhalf/PO", 2000, &MoCapMessenger::callbackFunction, &MocapRobulab);
   MocapGoalLeft.sub = MocapGoalLeft.n.subscribe("/evart/goal_left/PO", 2000, &MoCapMessenger::callbackFunction, &MocapGoalLeft);
   MocapGoalRight.sub = MocapGoalRight.n.subscribe("/evart/goal_right/PO", 2000, &MoCapMessenger::callbackFunction, &MocapGoalRight);

   // Init ROS
   ros::NodeHandle node_message;

   // To receive data from vision control
   ros::Subscriber _subscriber = node_message.subscribe("datacircles", 2000, &getDataCircles);

   // To send data to Matlab
   ros::Publisher  _publisherrobotcontrol = node_message.advertise<std_msgs::Float64MultiArray>("VelocityControl", 2000);
   ros::Publisher  _MocapRobulabPublisher = node_message.advertise<std_msgs::Float64MultiArray>("RobotState", 2000);
   ros::Publisher  _MocapGoalPublisher = node_message.advertise<std_msgs::Float64MultiArray>("GoalState", 2000);

   // Open the connection with the Robot Robulab
   Robot.establish_connection();

   // Active signal (CTRL + C interrupt)
   struct sigaction sigIntHandler;
   sigIntHandler.sa_handler = my_handler;
   sigemptyset(&sigIntHandler.sa_mask);
   sigIntHandler.sa_flags = 0;
   sigaction(SIGINT, &sigIntHandler, NULL);

   /// Elliptical Coordinates Control Feedback

   double x1, y1, x2, y2, A, B, C, xi, eta, betae;
   double x1centered, y1centered, x2centered, y2centered;
   double x1centered_before, y1centered_before, x2centered_before, y2centered_before;

   double omega, omegao;
   double v, w;
   bool goal_reached = false;
   int iter=0;

   v = 0;    w = 0;

   /// ROS data message

   std::vector<double> robotcontrol_data_t;
   std::vector<std::vector<double> > robotcontol_data;
   std::vector<double> robotstate_t;
   std::vector<std::vector<double> > robotstate_data;
   std::vector<double> goalstate_t;
   std::vector<double> support_variable;
   std::vector<std::vector<double> > goalstate_data;
   std::vector<double> support_vector_position_switch_control;

   /// Parameter of the camera (defined by calibration)
   double alphax =  712.27744;
   double alphay =  711.92580;
   double PPx = 330.98367;        // Principal point X
   double PPy = 273.41515;        // Principal point Y
   double a = 70/2;
   double kc[5] = {-0.39852, 0.24318, 0.00180, -0.00011, 0.00000};
   double pixelxy[2];
   double cc[2] = {PPx, PPy};
   double fc[2] = {alphax, alphay};
   double alpha_c = 0;
   double xp[2];
   double x1p, y1p;
   double p1, p2, p3;

   double x1centereddist, x2centereddist, y1centereddist,y2centereddist;

   /// Gains of the control
   /*
   double K1 = 0.001;       // Gain for the forward velocity
   double K2 = 1;   // How fast beta converges to zero
   double lambda = 2000;    // How fast eta converges to zero
   */
   double K1, K2, lambda;
   /// Gains of the control
   K1 = atof(argv[2]);
   K2 = atof(argv[3]);
   lambda =  atof(argv[4]);

   /// Delay before to get information from MocapRobulab (to avoid null data)
   std::cout << "... Initilization Virtual Tracking..." << std::endl;

   int samples = 0.1/0.02; // 1 second divided by 20ms
   double Ka[samples];

   for (int i=1; i<=samples; ++i){
     Ka[i] = (double)i/samples;
     }
   std::cout << "Parameters initialized... " << std::endl;

   /// Starting to call the communication
   double timenowdouble = time(NULL)+1;
   while(time(NULL)<timenowdouble){
       ros::spinOnce();
       ros::Duration(0.02).sleep();
     }

   bool target_lost = false;
   bool go_though_door = false;
   double iter_error = 0;
   iter=1;
   /// TRACK CONTROL
   while(ros::ok() && goal_reached == false && stop_interrupt==false ){

       ros::spinOnce();


       /// Take data from MocapRobulab, tracking the Robot
       robotstate_t.clear();
       robotstate_t = MocapRobulab.item_XY_YAW_configuration_OFFSET(-2.23393);
       robotstate_data.push_back(robotstate_t);

       goalstate_t.clear();
       support_variable.clear();

       goalstate_t = MocapGoalLeft.item_XY_YAW_configuration();
       support_variable = MocapGoalRight.item_XY_YAW_configuration();

       /// Concatenate vectors such that the vector is [x1 y1 t1 x2 y2 t2]
       goalstate_t.insert( goalstate_t.end(), support_variable.begin(), support_variable.end() );

       goalstate_data.push_back(goalstate_t);

       std::cout << "Object: x " << goalstate_t[0] << " y " << goalstate_t[1] << "th " << goalstate_t[2] << std::endl;
       std::cout << "Object: x " << goalstate_t[3] << " y " << goalstate_t[4] << "th " << goalstate_t[5] << std::endl;

       /// Elliptic coordinates computed from the image plane measurements
       /// Input parser...(feature points)

       target_lost = false;

       /// SECURITY CHECK
       if(CirclesData[0][0]==CirclesData[1][0])
       {
        target_lost = true;
        std::cout << "Lost one "<< std::endl;
       }

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


/*
       if(CirclesData[0][0]>CirclesData[1][0])
         {
           x1centereddist = CirclesData[1][0] - PPx;
           y1centereddist = CirclesData[1][1] - PPy;
           x2centereddist = CirclesData[0][0] - PPx;
           y2centereddist = CirclesData[0][1] - PPy;
         }
       else{
           x1centereddist = CirclesData[0][0] - PPx;
           y1centereddist = CirclesData[0][1] - PPy;
           x2centereddist = CirclesData[1][0] - PPx;
           y2centereddist = CirclesData[1][1] - PPy;
         }
       //void remove_distorsion(double *pixelxy, double *cc, double *fc, double *kc, double alpha_c, double *xp)

        pixelxy[0]=x1centereddist  ;
        pixelxy[1]=y1centereddist  ;
        remove_distorsion(pixelxy, cc, fc, kc, alpha_c, xp);
        x1centered = xp[0];
        y1centered = xp[1];

        pixelxy[0]=x2centereddist  ;
        pixelxy[1]=y2centereddist  ;
        remove_distorsion(pixelxy, cc, fc, kc, alpha_c, xp);
        x2centered = xp[0];
        y2centered = xp[1];
*/
       /// If the targets or just one are not detected

       if((x1centered==x1centered_before && y1centered==y1centered_before) || (x2centered==x2centered_before && y2centered==y2centered_before))
         {
           std::cout << "Equal data" << std::endl;
           target_lost = true;
          }
       if((iter % 200)==0){
         x1centered_before = x1centered;
         x2centered_before = x2centered;
         y1centered_before = y1centered;
         y2centered_before = y2centered;
         }

       /// If the two target are not in the same level (one of them is noise)
       if(fabs(y1centered-y2centered)>10)
         {
          target_lost = true;
          std::cout << "Different LEVEL" << std::endl;
         }



       x1 = x1centered;
       y1 = (y1centered/fabs(y1centered))*y1centered;

       x2 = x2centered;
       y2 = (y2centered/fabs(y2centered))*y2centered;

       // Check ellipses position
       std::cout << " ---------------------------------- " << std::endl;
       std::cout << " x1c = " << x1centered << " x1cb " << x1centered_before << std::endl;
       std::cout << " x2c = " << x2centered << " x2cb " << x2centered_before << std::endl;
       std::cout << " y1c = " << y1centered << " y1cb " << y1centered_before << std::endl;
       std::cout << " y2c = " << y2centered << " y2cb " << y2centered_before << std::endl;
       std::cout << " ---------------------------------- " << std::endl;


       /// CONTROL LAW
       A = y2*sqrt(x1*x1+alphax*alphax);
       B = y1*sqrt(x2*x2+alphax*alphax);
       C = sqrt((x1*y2-x2*y1)*(x1*y2-x2*y1)+alphax*alphax*(y2-y1)*(y2-y1));

       /// Elliptic coordinates
       xi = acosh((A+B)/C);
       eta = M_PI/2-acos((A-B)/C);
       betae = -(atan(x1/alphax)+(atan(x2/alphax)-atan(x1/alphax))/2);

       /// Control law in elliptic coordinates (Path Following like)
       if(iter<samples){
         //v = a * Ka[iter] * K1 * cos(eta) * cosh(xi) * sqrt(1+tan(eta) * tan(eta) * tanh(xi) * tanh(xi));
         v = a * K1 * cos(eta) * cosh(xi) * sqrt(1+tan(eta) * tan(eta) * tanh(xi) * tanh(xi));
         }
       else
         v = a * K1 * cos(eta) * cosh(xi) * sqrt(1+tan(eta) * tan(eta) * tanh(xi) * tanh(xi));

       if (v>1.5){
           Robot.move_robot(0,0);
           std::cout << "OHHHH DOVE VAIII " << v << std::endl;
           exit(-1);
         }


       if (fabs(betae)<=0.001)
           omegao = -lambda*eta*K1-K2*betae;
       else
           omegao = -lambda*eta*K1*sin(betae)/betae-K2*betae;

       p1 = cos(2*eta)+cosh(2*xi);
       p2 = cos(betae)*cos(eta)*sin(eta);
       p3 = sin(betae)*sinh(2*xi);

       omega = -omegao+K1*(-2*p2+p3)/p1;

       w = omega;

        // Some print out
       std::cout << " ----------------------------------------- " << std::endl;
       std::cout << "xi " << xi << std::endl;
       std::cout << "eta: " << eta << std::endl;
       std::cout << "betae: " << betae << std::endl;
       std::cout << "Robot controls - v: " << v << " omega: " << w <<  " omegao: " << omegao << std::endl;
       std::cout << "K1 " << K1 << " - K2 " << K2 << " - lambda " << lambda << std::endl;
       //std::cout << " ----------------------------------------- " << std::endl;

       /// Move ROBOT
       if((CirclesData[1][1]<20 || CirclesData[0][1]<20) || (CirclesData[1][1]>464 || CirclesData[0][1]>464)){
         Robot.move_robot(0, 0);
         if(CirclesData[0][0]!=0 && CirclesData[0][1]!=0)
          stop_interrupt=true;
          go_though_door=true;
         }
      else
         if(!target_lost){
           std::cout << "Move!" << std::endl;
           if(atof(argv[1])==1)
               Robot.move_robot(v, w);
           }
         else
           {
           iter_error++;
           std::cout << "Error detected. " << iter_error << std::endl;
           }

       if(iter_error>100){
         stop_interrupt=true;
         exit(-1);
         std::cout << "TARGET NOT VALID" << std::endl;
         }

       /// Create message to send to Matlab
       robotcontrol_data_t.push_back(v);
       robotcontrol_data_t.push_back(w);
       robotcontol_data.push_back(robotcontrol_data_t);
       robotcontrol_data_t.clear();

       support_vector_position_switch_control.clear();
       support_vector_position_switch_control = MocapRobulab.item_XY_YAW_configuration_OFFSET(-2.23393);

       ros::Duration(0.02).sleep();

       ++iter;
   }

   std::cout << "last pos change control " << support_vector_position_switch_control[0] << " " << support_vector_position_switch_control[1] << std::endl;


   double spaceoffset = 1.3;
   double timeoffset = spaceoffset / v;


   if(go_though_door){
       stop_interrupt=false;
     std::cout << "Through the door" << std::endl;
     timenowdouble = time(NULL)+timeoffset;
     while(time(NULL)<timenowdouble && stop_interrupt==false){
         ros::spinOnce();
         Robot.move_robot(v,0);


         /// Take data from MocapRobulab, tracking the Robot
         robotstate_t.clear();
         robotstate_t = MocapRobulab.item_XY_YAW_configuration_OFFSET(-2.23393);
         robotstate_data.push_back(robotstate_t);

         robotcontrol_data_t.clear();
         robotcontrol_data_t.push_back(v);
         robotcontrol_data_t.push_back(0);
         robotcontol_data.push_back(robotcontrol_data_t);



         /// Concatenate vectors such that the vector is [x1 y1 t1 x2 y2 t2]
         goalstate_t.insert( goalstate_t.end(), support_variable.begin(), support_variable.end() );

         goalstate_data.push_back(goalstate_t);

         ros::Duration(0.02).sleep();
       }
    }

   Robot.move_robot(0, 0);



   robotstate_data.push_back(support_vector_position_switch_control);

   goalstate_t.clear();
   goalstate_t.push_back(K1);
   goalstate_t.push_back(K2);
   goalstate_t.push_back(-1);
   goalstate_t.push_back(lambda);
   goalstate_t.push_back(lambda);
   goalstate_t.push_back(-1);

   goalstate_data.push_back(goalstate_t);

   std::cout << "Converting Data RobotControl to Matlab" << std::endl;
   Eigen::MatrixXd robotcontrol_data_matrix = Eigen::MatrixXd::Zero(robotcontol_data.size(),2);
   std_msgs::Float64MultiArray robotcontrol_data_msg;

   for(unsigned int c=0; c<robotcontol_data.size(); ++c){
      robotcontrol_data_matrix(c,0) = robotcontol_data[c][0];
      robotcontrol_data_matrix(c,1) = robotcontol_data[c][1];
   }

   std::cout << "Converting Data RobotState to Matlab" << std::endl;
   Eigen::MatrixXd robotstate_data_matrix = Eigen::MatrixXd::Zero(robotstate_data.size(),3);
   std_msgs::Float64MultiArray robotstate_data_msg;

   for(unsigned int c=0; c<robotstate_data.size(); ++c){
      robotstate_data_matrix(c,0) = robotstate_data[c][0];
      robotstate_data_matrix(c,1) = robotstate_data[c][1];
      robotstate_data_matrix(c,2) = robotstate_data[c][2];
   }

   std::cout << "Converting Data GoalState to Matlab" << std::endl;
   Eigen::MatrixXd goalstate_data_matrix = Eigen::MatrixXd::Zero(goalstate_data.size(),6);
   std_msgs::Float64MultiArray goalstate_data_msg;

   for(unsigned int c=0; c<goalstate_data.size(); ++c){
      goalstate_data_matrix(c,0) = goalstate_data[c][0];
      goalstate_data_matrix(c,1) = goalstate_data[c][1];
      goalstate_data_matrix(c,2) = goalstate_data[c][2];
      goalstate_data_matrix(c,3) = goalstate_data[c][3];
      goalstate_data_matrix(c,4) = goalstate_data[c][4];
      goalstate_data_matrix(c,5) = goalstate_data[c][5];
   }



   /// Define the message to send
   tf::matrixEigenToMsg(robotcontrol_data_matrix, robotcontrol_data_msg);
   tf::matrixEigenToMsg(robotstate_data_matrix,  robotstate_data_msg);
   tf::matrixEigenToMsg(goalstate_data_matrix,  goalstate_data_msg);

   std::cout << "Done. " << std::endl;
   for(unsigned int it=0; it<300; it++){
    _publisherrobotcontrol.publish(robotcontrol_data_msg);
    _MocapRobulabPublisher.publish(robotstate_data_msg);
    _MocapGoalPublisher.publish(goalstate_data_msg);

    ros::Duration(0.05).sleep();
    }

   Robot.move_robot(0, 0);

   return 0;

}
