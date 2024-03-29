#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <cmath>

visualization_msgs::Marker points, line_strip, line_list;


bool gonder=false;
void chatterCallback(const geometry_msgs::PoseArray& msg){

ROS_INFO("I heard:%d",msg.poses.size());
   
points.points.clear();
	line_strip.points.clear();
        
    for(int i=0; i<msg.poses.size(); i++)
{
   
    geometry_msgs::Point p;
    p.x = msg.poses[i].position.x;
    p.y = msg.poses[i].position.y;
    p.z = msg.poses[i].position.z+1.0;

    points.points.push_back(p);
    line_list.points.push_back(p);
    p.z = msg.poses[i].position.z;
    line_list.points.push_back(p);

}
gonder=true;

ROS_INFO("I heard:%d",points.points.size());
	
}



int main( int argc, char** argv )
{

    ros::init(argc, argv, "points_and_lines");
    
    ros::NodeHandle n;
ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Subscriber sub = n.subscribe("/object_geometry_blue",10,&chatterCallback);

    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
   
    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    
    points.scale.x = 0.2;
    points.scale.y = 0.2;
   
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;
    
    points.color.g = 1.0f;
    points.color.a = 1.0;
    
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

      ros::Rate rate(10);
      while (ros::ok())
      {    
	  
	
	  marker_pub.publish(points);
          marker_pub.publish(line_strip);
          marker_pub.publish(line_list);  
	
	
	
	  
          rate.sleep();
          ros::spinOnce();
      }
}
