#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#define voxel_x_max 0.10
#define voxel_x_min -0.10
#define voxel_y_max 0.10
#define voxel_y_min -0.10
#define voxel_z_max 0.90
#define voxel_z_min 0.70




class kinect_twist {

ros::NodeHandle nh;
ros::Subscriber sub_1;
ros::Publisher twist_pub;
geometry_msgs::Twist object_twist;
geometry_msgs::Point current_point;

public:

kinect_twist()
{
  twist_pub= nh.advertise<geometry_msgs::Twist>("object_twist", 10);
  sub_1=nh.subscribe<geometry_msgs::Point>("object_centre",1,&kinect_twist::twist_callback, this);
  
}
 
 private:

void twist_callback(const geometry_msgs::Point::ConstPtr& point_ptr)
{
  
    geometry_msgs::Point p;
    if(point_ptr->z==point_ptr->z)
    {
      
      if(point_ptr->x>voxel_x_max)
        p.y=(point_ptr->x-0.10)*15;

      if(point_ptr->x<voxel_x_min)
        p.y=(point_ptr->x+0.10)*15;

      if(point_ptr->y>voxel_y_max)
        p.z=(point_ptr->y-0.10)*15;

      if(point_ptr->y<voxel_y_min)
        p.z=(point_ptr->y+0.10)*15;

      if(point_ptr->z>voxel_z_max)
        p.x=(-point_ptr->z+0.90)*15;

      if(point_ptr->z<voxel_z_min)
        p.x=(-point_ptr->z+0.70)*15;
      
      
      current_point=p;

      if(!((point_ptr->x<voxel_x_max) && (point_ptr->x>voxel_x_min) && (point_ptr->y<voxel_y_max) && (point_ptr->y>-voxel_y_min) && (point_ptr->z<voxel_z_min) && (point_ptr->z>voxel_z_max)))
        if(abs(p.x)<0.80 && abs(p.y)<0.80 && abs(p.z)<0.80)
        publish_twist();
    
    }
    

    

}

void publish_twist()
{
    
     object_twist.linear.x=current_point.x;
     object_twist.linear.y=current_point.y;
     object_twist.linear.z=current_point.z;
     object_twist.angular.x=0;
     object_twist.angular.y=0;
     object_twist.angular.z=0;
     
     twist_pub.publish(object_twist);
     ROS_INFO_STREAM("Velocities:"<<"x:"<<object_twist.linear.x);
     ROS_INFO_STREAM("Velocities:"<<"y:"<<object_twist.linear.y);
     ROS_INFO_STREAM("Velocities:"<<"z:"<<object_twist.linear.z);

    
    
}

};


int main (int argc, char** argv)
{
  ros::init(argc,argv,"kinect_twist");

  kinect_twist twist;
  
  ros::spin();
}