
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <stdio.h>
#include <iostream>

#include "CameraClass.hpp"

using namespace std;

int main(int argc, char **argv){

    ros::init(argc, argv, "ellipsedet");
    ros::start();

    Camera Cam;
    int counting;
    Mat src, frame;
    vector<Vec3f> circles;
    Point2f rectangles[4];
    vector<RotatedRect> minEllipse;

    VideoCapture vcap = Cam.camera_connection("rtsp://192.168.1.99/live.sdp");

    while(ros::ok()) {
        vcap >> src; // get a new frame from camera

        if(!vcap.read(frame)) {
            std::cout << "No frame" << std::endl;
            waitKey();
        }

        /// Ellipses detection
        Cam.ellipsedetection(src, minEllipse, &counting);
        Cam.circledetection(src, circles);

        if (circles.size()>0)
        std::cout << circles[0] << std::endl;

        std::cout << "counting " << counting << std::endl;
        for(unsigned i=0; i<counting; ++i)
          std::cout << "ellispse " << i << " " << minEllipse[i].center << std::endl;

    }
}





