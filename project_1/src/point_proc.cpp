#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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
#include <geometry_msgs/Point.h>


ros::Publisher pub_1;
ros::Publisher pub_2;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
int count=0;

void point_cb(const pcl::PCLPointCloud2ConstPtr& pcl_ptr)
{
	//pass through filtering.
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PCLPointCloud2 cloud_out;
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::fromPCLPointCloud2(*pcl_ptr, *cloud_xyz);
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PassThrough<pcl::PointXYZ> pass;
 pass.setInputCloud (cloud_xyz);
 pass.setFilterFieldName ("x");
 pass.setFilterLimits (-0.50,0.50);
 pass.filter (*cloud_pass);
 pass.setInputCloud (cloud_pass);
 pass.setFilterFieldName ("y");
 pass.setFilterLimits (-0.50,0.50);
 pass.filter (*cloud_pass);
 pass.setInputCloud (cloud_pass);
 pass.setFilterFieldName ("z");
 pass.setFilterLimits (0,1.8);
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

if(count>0)
{
//Change Estimator
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(128.0f);
 
	// Add cloudA to the octree.
	octree.setInputCloud(cloudA);
	octree.addPointsFromInputCloud();
	// The change detector object is able to store two clouds at the same time;
	// with this we can reset the buffer but keep the previous tree saved.
	octree.switchBuffers();
	// Now, add cloudB to the octree like we did before.
	octree.setInputCloud(cloud_pass);
	octree.addPointsFromInputCloud();
 
	std::vector<int> newPoints;
	// Get a vector with the indices of all points that are new in cloud B,
	// when compared with the ones that existed in cloud A.
	octree.getPointIndicesFromNewVoxels(newPoints);
	cloud_final->height=1;
	cloud_final->width=newPoints.size();
	cloud_final->points.resize(newPoints.size());

	for (size_t i = 0; i < newPoints.size(); ++i)
	{
		cloud_final->points[i].x=cloud_pass->points[newPoints[i]].x;
		cloud_final->points[i].y=cloud_pass->points[newPoints[i]].y;
		cloud_final->points[i].z=cloud_pass->points[newPoints[i]].z;

	}

	//computing the centroid
   	Eigen::Vector4f centroid;
   	geometry_msgs::Point obj_centroid;
    pcl::compute3DCentroid(*cloud_final, centroid);
    obj_centroid.x=centroid[0];
    obj_centroid.y=centroid[1];
    obj_centroid.z=centroid[2];
    ROS_INFO_STREAM("The centroid of the moving object:"<<centroid[0]<<","<<centroid[1]<<","<<centroid[2]);
    pub_2.publish(obj_centroid);

 }

 cloudA=cloud_pass;
 count=count+1;
 
 pcl::toPCLPointCloud2(*cloud_final,cloud_out);
 cloud_out.header.frame_id=pcl_ptr->header.frame_id;
 cloud_out.header.stamp=pcl_ptr->header.stamp;
 pub_1.publish(cloud_out);
}






int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_proc");
  ros::NodeHandle nh;
    
    ros::Subscriber sub_1;
    pub_1=nh.advertise<sensor_msgs::PointCloud2>("pointcloud/object", 1);
    pub_2=nh.advertise<geometry_msgs::Point>("object_centre",1);
    sub_1=nh.subscribe<pcl::PCLPointCloud2>("camera/depth_registered/points", 1,point_cb );

  
  ros::spin();
}