#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <cmath>


class Visualization {

ros::NodeHandle nh;
ros::Subscriber sub_1;
ros::Publisher marker_pub;
ros::Publisher marker_pub_1;
ros::Publisher marker_pub_2;
ros::Publisher marker_pub_3;


int count=0;
visualization_msgs::Marker points, line_strip, line_list;
visualization_msgs::Marker marker;
visualization_msgs::Marker marker_1;
visualization_msgs::Marker marker_2;


public:

Visualization()
{
  marker_pub= nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  sub_1=nh.subscribe<geometry_msgs::Point>("object_centre",1,&Visualization::point_callback, this);
  marker_pub_1=nh.advertise<visualization_msgs::Marker>("visualization_marker_1",1);
  marker_pub_2=nh.advertise<visualization_msgs::Marker>("visualization_marker_2",1);
  marker_pub_3=nh.advertise<visualization_msgs::Marker>("visualization_marker_3",1);


}
 
 private:

void point_callback(const geometry_msgs::Point::ConstPtr& point_ptr)
{
  
     
    //path description
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/camera_depth_frame";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    
//control voxel visualization
marker.header.frame_id = "/camera_depth_frame";
marker.header.stamp = ros::Time();
marker.ns = "points_and_lines";
marker.id = 0;
marker.type = visualization_msgs::Marker::CUBE;
marker.action = visualization_msgs::Marker::ADD;
marker.pose.position.x = 0.80;
marker.pose.position.y = 0;
marker.pose.position.z = 0;
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;
marker.scale.x = 0.20;
marker.scale.y = 0.20;
marker.scale.z = 0.20;
marker.color.a = 1.0; // Don't forget to set the alpha!
marker.color.r = 0.8;
marker.color.g = 0.2;
marker.color.b = 0.0;

//kinect camera visualisation
marker_1.header.frame_id = "/camera_depth_frame";
marker_1.header.stamp = ros::Time();
marker_1.ns = "points_and_lines";
marker_1.id = 0;
marker_1.type = visualization_msgs::Marker::CUBE;
marker_1.action = visualization_msgs::Marker::ADD;
marker_1.pose.position.x = 0;
marker_1.pose.position.y = 0;
marker_1.pose.position.z = 0;
marker_1.pose.orientation.x = 0.0;
marker_1.pose.orientation.y = 0.0;
marker_1.pose.orientation.z = 0.0;
marker_1.pose.orientation.w = 1.0;
marker_1.scale.x = 0.05;
marker_1.scale.y = 0.10;
marker_1.scale.z = 0.05;
marker_1.color.a = 1.0; // Don't forget to set the alpha!
marker_1.color.r = 0.2;
marker_1.color.g = 0.0;
marker_1.color.b = 1.0;


    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;



    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;



    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.02;
    points.scale.y = 0.02;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.02;
    line_list.scale.x = 0.02;



    // Points are green
    points.color.g = 1.0;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    
    geometry_msgs::Point p=*point_ptr;
    if(p.z==p.z&&p.z!=0)
    {
      //coordinate transformation kinect->robot
      p.x=(-point_ptr->z);
      p.y=(-point_ptr->x);
      p.z=(-point_ptr->y);
    points.points.push_back(p);
    line_strip.points.push_back(p);

      // The line list needs two points for each line
    //line_list.points.push_back(p);
      
    //p.z += 0.2;
    //line_list.points.push_back(p);
    //count+=1;

    //hand position visualization
marker_2.header.frame_id = "/camera_depth_frame";
marker_2.header.stamp = ros::Time();
marker_2.ns = "points_and_lines";
marker_2.id = 0;
marker_2.type = visualization_msgs::Marker::SPHERE;
marker_2.action = visualization_msgs::Marker::ADD;
marker_2.pose.position.x = (-p.x);
marker_2.pose.position.y = p.y;
marker_2.pose.position.z = p.z;
marker_2.pose.orientation.x = 0.0;
marker_2.pose.orientation.y = 0.0;
marker_2.pose.orientation.z = 0.0;
marker_2.pose.orientation.w = 1.0;
marker_2.scale.x = 0.10;
marker_2.scale.y = 0.10;
marker_2.scale.z = 0.10;
marker_2.color.a = 1.0;
marker_2.color.r = 0.7;
marker_2.color.g = 0.7;
marker_2.color.b = 0.7;

  }
    
    publish_markers();

    
    

}

void publish_markers()
{
//    if(count==2)
  //  {
    marker_pub_3.publish(marker_2);
    marker_pub_2.publish(marker_1);
    marker_pub_1.publish(marker);
    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    count=0;
    //}
}

};


int main (int argc, char** argv)
{
  ros::init(argc,argv,"path_viz");

  Visualization viz;
  
  ros::spin();
}