#ifndef CAMERACLASS_HPP
#define CAMERACLASS_HPP

#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;

class Camera
{
public:
    Camera(){}
    ~Camera(){}
    VideoCapture camera_connection(const std::string videoStreamAddress);
   // void ellipsedetection(Mat src, Point2f *rect_points, int *counting);
    void ellipsedetection(Mat src, vector<RotatedRect> &minEllipse, int *counting);
    void circledetection(Mat src, vector<Vec3f> &circles);
    void remove_distorsion(double &pixelxy, double &cc, double &fc, double &kc, double alpha_c, double &xp);
};


VideoCapture Camera::camera_connection(const std::string videoStreamAddress)
{
  VideoCapture VideoStreaming;

  //open the video stream and make sure it's opened
  if(!VideoStreaming.open(videoStreamAddress)) {
      std::cout << "Error opening video stream or file" << std::endl;
      exit(-1);
  }

  return VideoStreaming;
}


void Camera::ellipsedetection(Mat src, vector<RotatedRect> &minEllipse, int *counting){

  /// Image variables
  Mat src_hsv, imgHSV;

  /// Set initial parameter to detect the ellipses
  Mat threshold_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  int thresh = 100;
  RNG rng(12345);
  Point2f rect_points[4];

  /// Convert the image from RGB to HSV
  cvtColor( src, src_hsv, CV_RGB2HSV );

  /// Show the image in HSV version
  imshow( "Source", src_hsv );
  moveWindow("Source", 680, 200);

  /// Set filter parameters to detect the ellipse

  /// AFTERNOON
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


  /* MORNING
  int iLowH = 115;
  int iHighH = 145;

  int iLowS = 100;
  int iHighS = 200;

  int iLowV = 125;
  int iHighV = 150;


  //Create trackbars in "Control" window
  cvCreateTrackbar("LowH", "Control", &iLowH, 115); //Hue (0 - 179)
  cvCreateTrackbar("HighH", "Control", &iHighH, 145);

  cvCreateTrackbar("LowS", "Control", &iLowS, 100); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Control", &iHighS, 200);

  cvCreateTrackbar("LowV", "Control", &iLowV, 125); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);
*/

  blur( src_hsv, src_hsv, Size(3,3) );
  GaussianBlur( src_hsv, src_hsv, Size(5, 5), 0, 0 );

  /// Remove all the color non needed
  inRange(src_hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgHSV); //Threshold the image

  /// Detect edges using Threshold
  threshold( imgHSV, threshold_output, thresh, 255, THRESH_BINARY );

  /// Find contours
  findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Find the rotated rectangles and ellipses for each contour
  vector<RotatedRect> minRect( contours.size() );
  minEllipse.resize(contours.size());

  for( int i = 0; i < contours.size(); i++ )
     { minRect[i] = minAreaRect( Mat(contours[i]) );
       if( contours[i].size() > 5 )
         { minEllipse[i] = fitEllipse( Mat(contours[i]) ); }
     }

  /// Draw contours + rotated rects + ellipses
  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );

  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       // contour
       drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
       // ellipse
       ellipse( drawing, minEllipse[i], color, 2, 8 );
       // rotated rectangle
       minRect[i].points( rect_points );
       for( int j = 0; j < 4; j++ )
          line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
     }

  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );

  //std::cout << "xe1 " << (rect_points[0].x + rect_points[2].x)/2 << " - ye1 " << (rect_points[1].y + rect_points[3].y)/2 << std::endl;
  *counting =  contours.size() ;
  //std::cout << "counting " << *counting << std::endl;

  waitKey(5);
}

void Camera::circledetection(Mat src, vector<Vec3f> &circles){

  /// Image variables
  Mat src_hsv, imgHSV;

  /// Convert the image from RGB to HSV
  cvtColor( src, src_hsv, CV_RGB2HSV );

  /// Show the image in HSV version
  imshow( "Source", src_hsv );
  moveWindow("Source", 680, 200);

  /// Set filter parameters to detect the ellipse
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
  blur( src_hsv, src_hsv, Size(5,5) );
  //GaussianBlur( src_hsv, src_hsv, Size(5, 5), 0, 0 );

  /// Remove all the color non needed
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
/*
 std::cout << "Circles (x,y,r)" << std::endl;
 for(unsigned int p=0; p<circles.size(); ++p)
    std::cout << circles[p] << std::endl;
*/

  //imshow( "Hough Circle Transform Final", imgHSV );

  waitKey(5);

}








#endif // CAMERACLASS_HPP
