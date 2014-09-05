
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include <std_msgs/Float64MultiArray.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <highgui.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <visp/vpV4l2Grabber.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vp1394CMUGrabber.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>

#include "Robulab10Class.hpp"

using namespace cv;
static bool stop_interrupt = false;

void my_handler(int s){
           Robulab10 Robot;
           printf("Caught signal %d\n",s);
           stop_interrupt = true;

           Robot.establish_connection();
           Robot.move_robot(0,0);
}

void streaming_process(cv::VideoCapture vcap, ros::Publisher* _blob_publisher)
{

  Mat src;
  vector<Vec3f> circles;
  std_msgs::Float64MultiArray cog_blobs;
  Eigen::MatrixXd blobs_matrix_data = Eigen::MatrixXd::Zero(2,3);

  /// Ellipses are detected considering rectangle shape
  Point2f rectangles[4];

  int counting = 0;
  int eps = 5;

  while(counting<2){

    /// OPENCV PART - Looking for ellipses

    vcap >> src; // get a new frame from camera

    if(!vcap.read(frame)) {
        std::cout << "No frame" << std::endl;
        waitKey();
    }

    /// Ellipses detection
    Cam.ellipsedetection(src,rectangles);

    if(circles.size()>0){
      if(circlesdatatemp.size()>0){
        if(!((fabs(circlesdatatemp[0]-circles[0][0])<eps)&&(fabs(circlesdatatemp[1]-circles[0][1]<eps)))){
            counting++;
          }
        }
      else{
        circlesdatatemp.push_back(circles[0][0]);
        circlesdatatemp.push_back(circles[0][1]);
        counting++;
        }
      }

    std::cout << "Looking for circles ( " << counting << " found) " << std::endl;
}

  /* DEBUGGING
  std::cout << "final counter" << counting << std::endl;
  std::cout << "final circles " << std::endl;
  for(unsigned int p=0; p<circles.size(); ++p)
     std::cout << circles[p] << std::endl;

  std::cout << "final storage ";
  std::cout << circlesdatatemp[0] << " " << circlesdatatemp[1] << std::endl;
*/

  /// VISP PART: Tracking circles
  /// Once found out the circles, tell visp where they are

  vpImage<unsigned char> I; // for gray images
  vpImagePoint vImp;
  vpImagePoint vImp2;

  vpImageConvert::convert(frame, I);  // convert Image from opencv to visp

  vpDisplayOpenCV d(I);
  vpDisplay::display(I);
  vpDisplay::flush(I);
  std::list<vpDot2> blob_list;
  vpDot2 blob;
  vpDot2 blob2;

  // assign blobs position
  vImp.set_u(circles[0][0]);
  vImp.set_v(circles[0][1]);
  if(circles.size()>1){
      vImp2.set_u(circles[1][0]);
      vImp2.set_v(circles[1][1]);
    }
  else{
      vImp2.set_u(circlesdatatemp[0]);
      vImp2.set_v(circlesdatatemp[1]);
    }

  // blobs definition
  blob.setGraphics(true);
  blob.setGraphicsThickness(1);
  blob.initTracking(I,vImp);

  blob.track(I);

  blob2.setGraphics(true);
  blob2.setGraphicsThickness(1);
  blob2.initTracking(I,vImp2);

  blob2.track(I);

  //   std::cout << "cog " << blob.getCog() << std::endl;
  //   std::cout << "cog2 " << blob2.getCog() << std::endl;

  double PPx = 330.98367;        // Principal point X
  double PPy = 273.41515;        // Principal point Y

  while(!stop_interrupt) {

    vcap >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);

    vpDisplay::display(I);
    blob.track(I);
    blob2.track(I);
    vpDisplay::displayCircle(I,PPy,PPx,10,vpColor::red) ;
    vpDisplay::displayCross(I, PPy,PPx, 20, vpColor::green) ;
    vpDisplay::flush(I);


    /// Publishing

    // blobs1 x (u)
    blobs_matrix_data(0,0) = blob.getCog().get_u();
    // blobs1 y (v)
    blobs_matrix_data(0,1) = blob.getCog().get_v();
    // blobs1 r
    blobs_matrix_data(0,2) = blob.getWidth()/2; // ray

    // blobs2 x (u)
    blobs_matrix_data(1,0) = blob2.getCog().get_u();
    // blobs2 y (v)
    blobs_matrix_data(1,1) = blob2.getCog().get_v();
    // blobs2 r
    blobs_matrix_data(1,2) = blob2.getWidth()/2; // ray
/*
    // blobs1 x (u)
    blobs_matrix_data(0,0) = 0;
    // blobs1 y (v)
    blobs_matrix_data(0,1) = 1;
    // blobs1 r
    blobs_matrix_data(0,2) = 2; // ray

    // blobs2 x (u)
    blobs_matrix_data(1,0) = 10;
    // blobs2 y (v)
    blobs_matrix_data(1,1) = 11;
    // blobs2 r
    blobs_matrix_data(1,2) = 12; // ray
*/
    // conversion to std_msgs
    tf::matrixEigenToMsg(blobs_matrix_data,cog_blobs);

    // Define the message to send
    _blob_publisher->publish(cog_blobs);

    std::cout << blobs_matrix_data << std::endl;

    ros::Duration(0.005).sleep();
  }
}

int main(int argc, char **argv)
{
    // Init ROS
    ros::init(argc, argv, "visioncontrol");
    ros::start();
    std::cout << "dentro main";

    ros::NodeHandle blobs;
    ros::Publisher _blob_publisher = blobs.advertise<std_msgs::Float64MultiArray>("datacircles", 2000);

    /// Establish connection with the camera streaming
    cv::VideoCapture vcap;
    const std::string videoStreamAddress = "rtsp://192.168.1.99/live.sdp";  /// insert here the address
    //const std::string videoStreamAddress;  /// insert here the address

    //open the video stream and make sure it's opened
    if(!vcap.open(videoStreamAddress)) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    std::cout << "quasi whilecccscs";

    /// Start Camera Process
    /// Circles are detected with OPENCV and then tracked with VISP.
    /// The topic provides pos_x, pos_y and ray dimension of the circles
    unsigned int exit_condition = 0;
    while(exit_condition<999){          // here condition of exit (when they are big, we can put some exit flag)
      try{
          std::cout << "dentro try";
        streaming_process(vcap, &_blob_publisher);
      }
      catch(...)
      {
        std::cout << "expection received nr. " << exit_condition << std::endl;
        exit_condition++;
     }
    }
    return 0;
}


