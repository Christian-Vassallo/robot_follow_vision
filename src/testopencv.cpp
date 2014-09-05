
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
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


using namespace cv;
/*
int main(int argc, char* argv[])
{
    CvCapture *camera=cvCaptureFromFile("rtsp://192.168.1.99/live.sdp");

    IplImage* frame = 0;
    CvMemStorage* storage = cvCreateMemStorage(0); //needed for Hough circles

    // capturing some extra frames seems to help stability
    frame = cvQueryFrame(camera);
    frame = cvQueryFrame(camera);
    frame = cvQueryFrame(camera);

    // with default driver, PSEye is 640 x 480
    CvSize size = cvSize(640,480);
    IplImage *  hsv_frame    = cvCreateImage(size, IPL_DEPTH_8U, 3);
    IplImage*  thresholded    = cvCreateImage(size, IPL_DEPTH_8U, 1);
    IplImage*  thresholded2    = cvCreateImage(size, IPL_DEPTH_8U, 1);

    CvScalar hsv_min = cvScalar(0, 50, 170, 0);
    CvScalar hsv_max = cvScalar(10, 180, 256, 0);
    CvScalar hsv_min2 = cvScalar(170, 50, 170, 0);
    CvScalar hsv_max2 = cvScalar(256, 180, 256, 0);

    do {
        frame = cvQueryFrame(camera);
        if (frame != NULL) {
            //printf("got frame\n\r");
            // color detection using HSV
            cvCvtColor(frame, hsv_frame, CV_BGR2HSV);
            // to handle color wrap-around, two halves are detected and combined
            cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);
            cvInRangeS(hsv_frame, hsv_min2, hsv_max2, thresholded2);
            cvOr(thresholded, thresholded2, thresholded);

            cvSaveImage("thresholded.jpg",thresholded);

            // hough detector works better with some smoothing of the image
            cvSmooth( thresholded, thresholded, CV_GAUSSIAN, 9, 9 );
            CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2, thresholded->height/4, 100, 40, 20, 200);

                    for (int i = 0; i < circles->total; i++)
                    {
                        float* p = (float*)cvGetSeqElem( circles, i );
                        //printf("Ball! x=%f y=%f r=%f\n\r",p[0],p[1],p[2] );
                            cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),
                                             3, CV_RGB(0,255,0), -1, 8, 0 );
                            cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),
                                             cvRound(p[2]), CV_RGB(255,0,0), 3, 8, 0 );
                     }

            //cvSaveImage("frame.jpg", frame);
            cvShowImage("img",frame);
            waitKey(33);

        } else {
            //printf("Null frame\n\r");
        }
  } while (true);
  cvReleaseCapture(&camera);
  return 0;
}
*/

int main(int argc, char **argv){

    ros::init(argc, argv, "testopencv");
    ros::start();

    VideoCapture vcap;
    Mat frame;
    Mat src, src_gray, src_hsv, imgHSV;

    // This works on a D-Link CDS-932L
    //const std::string videoStreamAddress = "http://192.168.1.99/cgi-bin/video.jpg";
    const std::string videoStreamAddress = "rtsp://192.168.1.99/live.sdp";

    //open the video stream and make sure it's opened
    if(!vcap.open(videoStreamAddress)) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    IplImage* frames;

    while(ros::ok()) {
        vcap >> src; // get a new frame from camera
        vector<Vec3f> circles;

        if(!vcap.read(frame)) {
            std::cout << "No frame" << std::endl;
            waitKey();
        }
        //src = imread("http://192.168.1.99/cgi-bin/video.jpg", CV_LOAD_IMAGE_COLOR);


        if( !src.data )
          { return -1; }

        cvtColor( src, src_hsv, CV_RGB2HSV );

        imshow( "Source", src_hsv );
        moveWindow("Source", 680, 200);
/*
        int iLowH = 0;
        int iHighH = 50;

        int iLowS = 75;
        int iHighS = 255;

        int iLowV = 0;
        int iHighV = 100;
        */

        /*
        office parameters

        int iLowH = 115;
        int iHighH = 145;

        int iLowS = 65;
        int iHighS = 142;

        int iLowV = 90;
        int iHighV = 170;

        //Create trackbars in "Control" window
        cvCreateTrackbar("LowH", "Control", &iLowH, 25); //Hue (0 - 179)
        cvCreateTrackbar("HighH", "Control", &iHighH, 25);

        cvCreateTrackbar("LowS", "Control", &iLowS, 75); //Saturation (0 - 255)
        cvCreateTrackbar("HighS", "Control", &iHighS, 75);

        cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
        cvCreateTrackbar("HighV", "Control", &iHighV, 255);

        GaussianBlur( src_hsv, src_hsv, Size(5, 5), 0, 0 );

        inRange(src_hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgHSV); //Threshold the image

        /// Apply the Hough Transform to find the circles
        HoughCircles( imgHSV, circles, CV_HOUGH_GRADIENT, 2.3, imgHSV.rows/4, 35, 43, 0, 0);

        */

        int iLowH = 115;
        int iHighH = 145;

        int iLowS = 100;
        int iHighS = 200;

        int iLowV = 90;
        int iHighV = 255;

        //Create trackbars in "Control" window
        cvCreateTrackbar("LowH", "Control", &iLowH, 25); //Hue (0 - 179)
        cvCreateTrackbar("HighH", "Control", &iHighH, 25);

        cvCreateTrackbar("LowS", "Control", &iLowS, 75); //Saturation (0 - 255)
        cvCreateTrackbar("HighS", "Control", &iHighS, 75);

        cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
        cvCreateTrackbar("HighV", "Control", &iHighV, 255);

        blur( src_hsv, src_hsv, Size(3,3) );
        GaussianBlur( src_hsv, src_hsv, Size(5, 5), 0, 0 );


        inRange(src_hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgHSV); //Threshold the image

        /// Apply the Hough Transform to find the circles
        HoughCircles( imgHSV, circles, CV_HOUGH_GRADIENT, 2.3, imgHSV.rows/4, 35, 30, 0, 0);

        /// Draw the circles detected
        for( size_t i = 0; i < circles.size(); i++ )
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle( src, center, 3, Scalar(0,255,0), -1, 0, 0 );
            // circle outline
            circle( src, center, radius, Scalar(0,0,255), 3, 0, 0 );
         }



        /// Show your results
        imshow( "Hough Circle Transform Final", src );
        moveWindow("Hough Circle Transform Final", 600, 0);

        imshow( "Original", imgHSV );
        moveWindow("Original", 0, 0);

       // std::cout << "Circles (x,y,r)" << std::endl;
       for(unsigned int p=0; p<circles.size(); ++p)
          std::cout << circles[p] << std::endl;


        //imshow( "Hough Circle Transform Final", imgHSV );

        waitKey(5);

    }

}




/** @function main */
/*
using namespace cv;

int main(int argc, char** argv)
{
  Mat src, src_gray, src_hsv, imgHSV;

  /// Read the image
  src = imread("pink.jpg", CV_LOAD_IMAGE_COLOR);
  imshow( "Hough Circle Transform Demo", src );

  if( !src.data )
    { return -1; }

  /// Convert it to gray
  cvtColor( src, src_hsv, CV_RGB2HSV );

  int iLowH = 110;
  int iHighH = 130;

  int iLowS = 50;
  int iHighS = 200;

  int iLowV = 50;
  int iHighV = 255;

  //Create trackbars in "Control" window
  cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
  cvCreateTrackbar("HighH", "Control", &iHighH, 179);

  cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Control", &iHighS, 255);

  cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);

  inRange(src_hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgHSV); //Threshold the image

  imshow( "Filtered", imgHSV );
  moveWindow("Filtered", 300, 500);

  vector<Vec3f> circles;

  /// Apply the Hough Transform to find the circles
  HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 2, src_gray.rows/4, 100, 50, 0, 0 );

  /// Draw the circles detected
  for( size_t i = 0; i < circles.size(); i++ )
  {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // circle center
      circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
      // circle outline
      circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );
   }


  /// Show your results
  imshow( "Hough Circle Transform Final", src );
  moveWindow("Hough Circle Transform Final", 600, 0);

  std::cout << "Circles (x,y,r)" << std::endl;
  for(unsigned int p=0; p<circles.size(); ++p)
    std::cout << circles[p] << std::endl;

  waitKey(0);


  return 0;
}
*/

