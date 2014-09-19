
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
#include <visp/vpMeLine.h>
#include <visp/vpMeEllipse.h>

#include "Robulab10Class.hpp"
#include "CameraClass.hpp"

using namespace cv;
static bool stop_interrupt = false;

void my_handler(int s){
           stop_interrupt = true;

           std::cout << "Process killed" << std::endl;
           exit(-1);
}


void streaming_process(cv::VideoCapture vcap, ros::Publisher* _blob_publisher)
{
  Camera Cam;

  int counting=0;                         // Number of ellipsoide detected
  int falsepositive=0;                    // Number of falsepositive ellipsoide detected
  int zerovalue=0;
  int eps = 5;                          // Threeshold to detect falsepositive
  bool notvalidvalue=false;

  // Active signal (CTRL + C interrupt)
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  Mat src;                              // Frame
  vector<Vec3f> circles;                // Vector of coordinates indicating x,y,r parameters of circles detected
  vector<RotatedRect> minEllipse;       // Vector of data indicating x,y coord of ellipses detected
  vector<RotatedRect> dataEllipses;

  std_msgs::Float64MultiArray cog_blobs;
  Eigen::MatrixXd blobs_matrix_data = Eigen::MatrixXd::Zero(2,3);

  /// Ellipses are detected considering rectangle shape

  while(counting<2){

    vcap >> src; // get a new frame from camera

    if(!vcap.read(src)) {
        std::cout << "No frame" << std::endl;
        waitKey();
    }

    /// Ellipses detection
    Cam.ellipsedetection(src, minEllipse, &counting);

    std::cout << "features detected: " << minEllipse.size() << std::endl;

    notvalidvalue = false;
    falsepositive = 0;
    zerovalue = 0;
    for(unsigned int n=0; n<counting; n++){


        for(unsigned int n2=0; n2<dataEllipses.size(); n2++){
            //std::cout << "compare (" << minEllipse[n].center.x << "," << minEllipse[n].center.y << ") con (" << dataEllipses[n2].center.x << "," << dataEllipses[n2].center.y << ")" << std::endl;
            if(fabs(minEllipse[n].center.x - dataEllipses[n2].center.x) < eps && fabs(minEllipse[n].center.y - dataEllipses[n2].center.y) < eps)
              {
              std::cout << "already detected" << std::endl;
              notvalidvalue = true;
              falsepositive++;
              break;
            }
          }
        if (notvalidvalue==false)
          {
          if(minEllipse[n].center.x!=0 && minEllipse[n].center.y!=0)
            dataEllipses.push_back(minEllipse[n]);
          else
            zerovalue++;
          }
      }

    std::cout << "false positive  " << falsepositive << std::endl;
    counting = counting - falsepositive - zerovalue;


    dataEllipses.clear();

    std::cout << "Real features detected: " << counting << std::endl;
    std::cout << " ------------ " << std::endl;



    }


  /// VISP PART: Tracking circles
  /// Once found out the circles, tell visp where they are

  Mat srcHSV, srcWB;
  vpImage<unsigned char> I; // for gray images
  vpImagePoint vImp;
  vpImagePoint vImp2;

    vpImageConvert::convert(src, I);  // convert Image from opencv to visp

  vpDisplayOpenCV d(I);
  vpDisplay::display(I);
  vpDisplay::flush(I);
  std::list<vpDot2> blob_list;
  vpDot2 blob;
  vpDot2 blob2;

  // assign blobs position
  vImp.set_u(dataEllipses[0].center.x);
  vImp.set_v(dataEllipses[0].center.y);
  vImp2.set_u(dataEllipses[1].center.x);
  vImp2.set_v(dataEllipses[1].center.y);

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

    vcap >> src; // get a new frame from camera
    vpImageConvert::convert(src, I);

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

    // conversion to std_msgs
    tf::matrixEigenToMsg(blobs_matrix_data,cog_blobs);

    // Define the message to send
    _blob_publisher->publish(cog_blobs);

    std::cout << "ellipse 1: " << blobs_matrix_data(0,0) << " - " << blobs_matrix_data(0,1) << std::endl;
    std::cout << "ellipse 2: " << blobs_matrix_data(1,0) << " - " << blobs_matrix_data(1,1) << std::endl;

  }

  ros::Duration(0.02).sleep();

}

int main(int argc, char **argv)
{
    // Init ROS
    ros::init(argc, argv, "visioncontrol");
    ros::start();
    std::cout << "dentro main";

    ros::NodeHandle blobs;
    ros::Publisher _blob_publisher = blobs.advertise<std_msgs::Float64MultiArray>("datacircles", 2000);

    Camera Cam;

    /// Establish connection with the camera streaming
    VideoCapture vcap = Cam.camera_connection("rtsp://192.168.1.99/live.sdp");

    /// Start Camera Process
    unsigned int exit_condition = 0;
    while(exit_condition<999 && ros::ok()){          // here condition of exit (when they are big, we can put some exit flag)
      try{
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



