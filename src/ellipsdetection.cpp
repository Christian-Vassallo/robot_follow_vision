
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

        std::cout << "counting " << std::endl;
        int eps = 10;

        for(unsigned c=0; c<counting-1; c++)
          std::cout << "ellipse " << c << " " <<  minEllipse[c].center << std::endl;

        for(unsigned n=0; n<counting-1; n++)
            for(unsigned n2=n+1; n2<counting; n2++)
              if(fabs(minEllipse[n].center.x - minEllipse[n2].center.x) < eps && fabs(minEllipse[n].center.y - minEllipse[n2].center.y) < eps)
                std::cout << "falso positivo" << std::endl;
    }
}





