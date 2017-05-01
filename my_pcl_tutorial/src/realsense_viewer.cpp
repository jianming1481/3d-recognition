#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>

ros::Publisher pub;
pcl::visualization::CloudViewer viewer("Cloud Viewer");

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	sensor_msgs::PointCloud2 output;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	output = *input;
	pcl::fromROSMsg(output,*cloud);
	viewer.showCloud(cloud);
	pub.publish (output);
}



int main (int argc, char** argv)
{ 


  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);
   ros::Rate loop_rate(100);
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
