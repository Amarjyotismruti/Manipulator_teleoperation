#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/octree/octree.h>
#include <pcl/common/centroid.h>
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

	ros::NodeHandle nh;
  ros::Publisher pub_1;
  ros::Publisher pub_2;
  ros::Subscriber sub_1;
  ros::Subscriber sub_2;
    

 public:

 	Object_track();

 private:

    void pointcloud_callback(const pcl::PCLPointCloud2ConstPtr& pcl_ptr);
    void image_callback(const sensor_msgs::ImageConstPtr& img_ptr);

};

Object_track::Object_track()
{
    pub_1=nh.advertise<sensor_msgs::PointCloud2>("pointcloud/object", 1);
    pub_2=nh.advertise<geometry_msgs::Point>("object_centre", 1);
    sub_1=nh.subscribe<pcl::PCLPointCloud2>("camera/depth_registered/points", 1, &Object_track::pointcloud_callback,this);
    sub_2=nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 1, &Object_track::image_callback,this);
    
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
  //median filtering
  cv::medianBlur(imgThresholded, imgThresholded,5);
}

void Object_track::pointcloud_callback(const pcl::PCLPointCloud2ConstPtr& pcl_ptr)
{
    
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;      // Convert PCL to XYZ data type
    pcl::fromPCLPointCloud2(*pcl_ptr, cloud_xyz);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_new->width=cloud_xyz.width;
    cloud_new->height=cloud_xyz.height;
    cloud_new->points.resize(cloud_new->width*cloud_new->height);
    int count=0;
    


    for(int j=0;j<cv_image.rows;j++) 
   {
     for (int i=0;i<cv_image.cols;i++)
     {

      if(cv_image.at<uchar>(j,i)==255 ) 
       {
    	cloud_new->points[count].x=cloud_xyz.points[count].x;
    	cloud_new->points[count].y=cloud_xyz.points[count].y;
    	cloud_new->points[count].z=cloud_xyz.points[count].z;
    	//ROS_INFO("Data assigned: %G", cloud_new.points[count].z );

       }  
       count++;        
     }
    }

    //pass through filtering.
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PCLPointCloud2 cloud_out;
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PassThrough<pcl::PointXYZ> pass;
 pass.setInputCloud (cloud_new);
 pass.setFilterFieldName ("x");
 pass.setFilterLimits (-0.50,0.50);
 pass.filter (*cloud_pass);
 pass.setInputCloud (cloud_pass);
 pass.setFilterFieldName ("y");
 pass.setFilterLimits (-0.50,0.50);
 pass.filter (*cloud_pass);
 pass.setInputCloud (cloud_pass);
 pass.setFilterFieldName ("z");
 pass.setFilterLimits (0,1.4);
 pass.filter (*cloud_pass);

 //remove NaN points.
std::vector<int> indices;
pcl::removeNaNFromPointCloud(*cloud_pass,*cloud_pass, indices);

//downsample via voxel grid filter
pcl::VoxelGrid<pcl::PointXYZ> vg;
vg.setInputCloud(cloud_pass);
vg.setLeafSize(0.03f,0.03f,0.03f);
vg.filter(*cloud_pass);

//outliner removal filter.
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
sor.setInputCloud (cloud_pass);
sor.setMeanK (50);
sor.setStddevMulThresh(1.0);
sor.filter(*cloud_pass);


//computing the centroid
    Eigen::Vector4f centroid;
    geometry_msgs::Point obj_centroid;
    pcl::compute3DCentroid(*cloud_pass, centroid);
    obj_centroid.x=centroid[0];
    obj_centroid.y=centroid[1];
    obj_centroid.z=centroid[2];
    ROS_INFO_STREAM("The centroid of the moving object:"<<centroid[0]<<","<<centroid[1]<<","<<centroid[2]);


//Scaling the centroid values for end-effector pose.


   
    pcl::toPCLPointCloud2(*cloud_pass, pcl_pointcloud);
    pcl_pointcloud.header.frame_id=pcl_ptr->header.frame_id;
    pcl_pointcloud.header.stamp=pcl_ptr->header.stamp;
    pub_1.publish(pcl_pointcloud);
    pub_2.publish(obj_centroid);
  
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Object_track");
  
  Object_track tracker;
  
  ros::spin();
}