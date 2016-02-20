#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
// PCL specific includes

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

class HSV_tracker
{
 public:
  int* HSV;
  ros::NodeHandle nh;
  ros::Subscriber sub_2;
 
 public:
  HSV_tracker(int* x)
  {
    HSV=x;
    sub_2=nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 1, &HSV_tracker::image_callback,this);
  }
 private:
  void image_callback(const sensor_msgs::ImageConstPtr& img_ptr);

};

void HSV_tracker::image_callback(const sensor_msgs::ImageConstPtr& img_ptr)
{
  cv::Mat cv_image;
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(img_ptr);
	cv_image=cv_ptr->image;
  cv::Mat imgHSV;
  cv::cvtColor(cv_image, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
  cv::Mat imgThresholded;
 
 int iLowH=cvGetTrackbarPos("LowH","Control");
 int iHighH=cvGetTrackbarPos("HighH","Control");
 int iLowS=cvGetTrackbarPos("LowS","Control");
 int iHighS=cvGetTrackbarPos("HighS","Control");
 int iLowV=cvGetTrackbarPos("LowV","Control");
 int iHighV=cvGetTrackbarPos("HighV","Control");
  

   cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
  //morphological opening (remove small objects from the foreground)
  cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );
  cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) ); 

   //morphological closing (fill small holes in the foreground)
  cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20)) ); 
  cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20)) );
   
   //calculating image moments
   cv::Moments oMoments = cv::moments(imgThresholded);
   double dM01 = oMoments.m01;
   double dM10 = oMoments.m10;
   double dArea = oMoments.m00;

   if (dArea > 10000)
  {
   //calculate the position of the ball
   int posX = dM10 / dArea;
   int posY = dM01 / dArea;  
      
   ROS_INFO_STREAM("The centroid of object:"<<posX<<","<<posY);
 }

  //median filtering
  cv::medianBlur(imgThresholded, imgThresholded,5);

    cv::imshow("WINDOW",imgThresholded);

  //contour detection
  int largest_area=0;
  int largest_contour_index=0;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Mat dst(cv_image.rows,cv_image.cols,CV_8UC1,cv::Scalar::all(0));
  //cv::Rect bounding_rect;
  cv::findContours(imgThresholded,contours,hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

  for (int i = 0; i < contours.size(); ++i)
  {
    double a=contourArea(contours[i],false);
    if(a>largest_area)
    {
      largest_area=a;
      largest_contour_index=i;

    }
  }
  //bounding_rect=boundingRect(contours[largest_contour_index]);
  cv::Scalar color(255,255,255);
  cv::drawContours(dst,contours,largest_contour_index,color,CV_FILLED,8,hierarchy);
  //cv::rectangle(cv_image, bounding_rect,  cv::Scalar(0,255,0),1, 8,0);
   // cv::imshow("WINDOW_1",cv_image);

  cv::waitKey(5);

  }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  
  cv::namedWindow("WINDOW", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("Control", CV_WINDOW_AUTOSIZE);
  int iLowH =0;
  int iHighH=20;

  int iLowS =0; 
  int iHighS=255;

  int iLowV =0;
  int iHighV=255;

  //Create trackbars in "Control" window
 cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
 cvCreateTrackbar("HighH", "Control", &iHighH, 179);

 cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 cvCreateTrackbar("HighS", "Control", &iHighS, 255);

 cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
 cvCreateTrackbar("HighV", "Control", &iHighV, 255);

 int HSV[6];
 HSV[0]=iLowH;
 HSV[1]=iHighH;
 HSV[2]=iLowS;
 HSV[3]=iHighS;
 HSV[4]=iLowV;
 HSV[5]=iHighV;
 
 HSV_tracker tracker(HSV);


  
  ros::spin();
}