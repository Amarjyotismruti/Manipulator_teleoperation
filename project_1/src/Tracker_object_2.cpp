#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include "project_1/vec_xyz.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <math.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
//openCV includes
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>


class Object_track
{
	cv::Mat cv_image;
	pcl::PCLPointCloud2 pcl_pointcloud;
  geometry_msgs::Point obj_loc;
  geometry_msgs::Pose obj_pose;
	ros::NodeHandle nh;
  ros::Publisher pub_1;
  ros::Publisher pub_2;
  ros::Publisher pub_3;
  ros::Subscriber sub_1;
  ros::Subscriber sub_2;
    

 public:

 	Object_track();

 private:

    void pointcloud_callback(const pcl::PCLPointCloud2ConstPtr& pcl_ptr);
    void image_callback(const sensor_msgs::ImageConstPtr& img_ptr);
    void publish_data();
};

Object_track::Object_track()
{
    pub_1=nh.advertise<sensor_msgs::PointCloud2>("pointcloud/object", 1);
    pub_2=nh.advertise<geometry_msgs::Pose>("object_pose", 1);
    pub_3=nh.advertise<geometry_msgs::Point>("object_centre", 1);
    sub_2=nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 1, &Object_track::image_callback,this);
    sub_1=nh.subscribe<pcl::PCLPointCloud2>("camera/depth_registered/points", 1, &Object_track::pointcloud_callback,this);
    
    
}

void Object_track::image_callback(const sensor_msgs::ImageConstPtr& img_ptr)
{
  cv::Mat imgRGB;
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(img_ptr);
	imgRGB=cv_ptr->image;
  cv::Mat imgHSV;

  cv::cvtColor(imgRGB, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
  cv::Mat imgThresholded;

 // HSV Values set for orange colored object
 int iLowH=100 ;
 int iHighH=130;
 int iLowS=96;
 int iHighS=145;
 int iLowV=81;
 int iHighV=119;
  

  cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
  //morphological opening (remove small objects from the foreground)
  cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );
  cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) ); 

   //morphological closing (fill small holes in the foreground)
  cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20)) ); 
  cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20)) );

  cv_image=imgThresholded;
    
}

void Object_track::pointcloud_callback(const pcl::PCLPointCloud2ConstPtr& pcl_ptr)
{
    
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;  
    pcl::PointCloud<pcl::PointXYZ> cloud_new_2;    // Convert PCL to XYZ data type
    pcl::fromPCLPointCloud2(*pcl_ptr, cloud_xyz);
    pcl::PointCloud<pcl::PointXYZ> cloud_new;
    cloud_new.width=cloud_xyz.width;
    cloud_new_2.width=cloud_xyz.width;
    cloud_new_2.height=cloud_xyz.height;
    cloud_new.height=cloud_xyz.height;
    cloud_new.points.resize(cloud_new.width*cloud_new.height);
    cloud_new_2.points.resize(cloud_new.width*cloud_new.height);
    int count=0;
    int count_2=0;
    float objx=0;
    float objy=0;
    float objz=0;
    double clm=cv_image.cols;
    bool invalid_point;
    

   //calculating image moments.
   cv::Moments oMoments = cv::moments(cv_image);
   double dM01 = oMoments.m01;
   double dM10 = oMoments.m10;
   double dArea = oMoments.m00;
   //classifying object based on area
   if (dArea > 10000)
  {
   //calculate the position of the ball.
   int posX = dM10 / dArea;
   int posY = dM01 / dArea;  
   ROS_INFO_STREAM("The centroid of object:"<<posX<<","<<posY);
   //Refining point cloud of invalid points.
   count=((posY)*clm+posX);
   float depth_object=cloud_xyz.points[count].z;
   bool invalid_point=depth_object==depth_object;

   //if(invalid_point==0)
   //
    //while(invalid_point==1)
    //{
      //count++;
      //float depth_object=cloud_xyz.points[count].z;
      //bool invalid_point=depth_object==depth_object;
    //}
   //}
   //assigning the pointcloud with the centre of the object.
   //cloud_new_2.points[count].x=objx;
   //cloud_new_2.points[count].y=objy;
   //cloud_new_2.points[count].z=objz;
   
   
    //extracting object point cloud.
    for(int j=posY-10;j<posY+10;j++) 
   {
     for (int i=posX-10;i<posX+10;i++)
     {

      if(cv_image.at<uchar>(j,i)==255 ) 
       {
        count=((j)*clm+i);
      cloud_new.points[count].x=cloud_xyz.points[count].x;
      cloud_new.points[count].y=cloud_xyz.points[count].y;
      cloud_new.points[count].z=cloud_xyz.points[count].z;
      //ROS_INFO("Data assigned: %G", cloud_new.points[count].z );

      //taking geometric mean of points//low pass filter
      invalid_point=cloud_xyz.points[count].z==cloud_xyz.points[count].z;
      if(!isnan(cloud_xyz.points[count].z))
      {
        objx=objx+cloud_xyz.points[count].x;
        objy=objy+cloud_xyz.points[count].y;
        objz=objz+cloud_xyz.points[count].z;
        count_2++;
        //ROS_INFO_STREAM("invalid:"<<cloud_xyz.points[count].x<<cloud_xyz.points[count].y<<cloud_xyz.points[count].z);
      }
       }  
             
     }
   }
   objx=objx/count_2;
   objy=objy/count_2;
   objz=objz/count_2;
 }

 //assigning pose values to point//co-ordinate transform.
   obj_loc.x=objx;
   obj_loc.y=objy;
   obj_loc.z=objz;
   float objy_gummi=objx;
   float objz_gummi=-objy;
   //zcentre shifted to 0.80
   float objx_gummi=0.80-objz;


   if(objy_gummi<0.10 && objy_gummi>-0.30 && objz_gummi<0 && objz_gummi>-0.30 && objx_gummi<0.30 && objx_gummi>0)
   {
   obj_pose.orientation.x=0;
   obj_pose.orientation.y=0;
   obj_pose.orientation.z=0;
   obj_pose.orientation.w=0;
   obj_pose.position.x=(objx_gummi);
   obj_pose.position.y=(objy_gummi);
   obj_pose.position.z=(objz_gummi);
   pub_2.publish(obj_pose);
   }





   cloud_new_2.points[count].x=objx;
   cloud_new_2.points[count].y=objy;
   cloud_new_2.points[count].z=objz;
   ROS_INFO_STREAM("Depth:"<<objz);

   ROS_INFO_STREAM("count_2:"<<obj_loc.z);
    //pcl::toPCLPointCloud2(cloud_new_2, pcl_pointcloud_point);
    pcl::toPCLPointCloud2(cloud_new, pcl_pointcloud);
    pcl_pointcloud.header.frame_id=pcl_ptr->header.frame_id;
    pcl_pointcloud.header.stamp=pcl_ptr->header.stamp;
    //pcl_pointcloud_point.header.frame_id=pcl_ptr->header.frame_id;
    //pcl_pointcloud_point.header.stamp=pcl_ptr->header.stamp;

    if(!isnan(objz))
      Object_track::publish_data();
    
  
}

void Object_track::publish_data()
{
  //pub_2.publish(pcl_pointcloud_point);
  pub_1.publish(pcl_pointcloud);
  pub_3.publish(obj_loc);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Object_track");
  
  Object_track tracker;
  
  ros::spin();
}