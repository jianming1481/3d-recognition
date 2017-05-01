#pragma once

#include "ros/ros.h"
#include "../include/OrganizedSegmentation.h"
#include "../include/CSHOTDescriptor.h"
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <string>
#include <sstream>
using namespace std;

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>


#include <boost/thread/thread.hpp>

#define SAVE_FULL_CLOUD

class SimpleRealSenseViewer
{
public:
    SimpleRealSenseViewer() : viewer("Cloud Viewer")
    {
        ROS_INFO("Initialize SimeleRealSenseViewer Class");

        frames_saved = 0;
        save_model = false;
		train_model = false;
		ncluster = 0;
    }

    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& Event,void* nothing) 
    { 
        if (Event.getKeySym() == "space" && Event.keyDown()) 
            std::cout<<"space"<<endl; 
    }

    boost::shared_ptr<pcl::visualization::CloudViewer> createViewer() 
    { 
		boost::shared_ptr<pcl::visualization::CloudViewer> v 
				(new pcl::visualization::CloudViewer("3D Viewer")); 
		v->registerKeyboardCallback<SimpleRealSenseViewer>(&SimpleRealSenseViewer::keyboardEventOccurred, *this); 

		return(v); 
    }

	void cloud_cb( const sensor_msgs::PointCloud2::ConstPtr& input){
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*input,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
        if (!viewer.wasStopped()) 
        {
            viewer.showCloud(cloud);
            if ( save_model ) 
			{
				OrganizedSegmentation seg;
				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr copycloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
				pcl::copyPointCloud(*cloud, *copycloud);
				ncluster = seg.FindSegmentPlaneScene(copycloud);

				seg.clusterViewer(copycloud);
				seg.SaveClusterModel(OUT_DIR, frames_saved);

				//save full cloud image
#ifdef SAVE_FULL_CLOUD
				pcl::PCDWriter writer1;
				std::stringstream ss1;
				ss1 << OUT_DIR << "cloud" << frames_saved << ".pcd";
				writer1.write<PointType> (ss1.str (), *cloud, false);
				ss1.str("");
				ss1.clear();
#endif
				
				save_model = false;
			}
        }
	}
	void SetInputDirectory(string outdir)
	{
		OUT_DIR = outdir;
		cout << "Directory is " << OUT_DIR << endl;
	}
    void run ()
	{

		char c;
		char data[2];
		long size = 0;
		int i = 1;
		while (!viewer.wasStopped())
		{
			c = getchar();
			if ( c == 's' )
			{
				frames_saved++;
				cout << "Saving clusters in frame " << frames_saved << ".\n";
				save_model = true;			
				sprintf(data, "%d", frames_saved);
			}
			else if ( c == 't' )
			{
				if (frames_saved > 0)
				{
					train_model = true;
				}
				else
				{
					cout << "Can't find clusters .\n";
				}			
			}
			if ( train_model )
			{
				int idx = 0;
				cout << "Input cluster index in frame " << frames_saved << ":\n";
				cin >> idx;

				CSHOTDescriptor des;
				des.DescriptorExtract(OUT_DIR, frames_saved, idx);
 
				cout << "cshot_" << frames_saved << "_" << idx << ".txt >> Save ok!\n";

				LoadClusterModel(OUT_DIR, frames_saved, idx);
				train_model = false;
			}
		}

	}

	void LoadClusterModel(std::string &OUT_PATH, int frames, int idx)
	{
		std::stringstream ss;

		pcl::PointCloud<PointType>::Ptr cluster (new pcl::PointCloud<PointType> ());
		ss << OUT_PATH.c_str() << "cloud_cluster_" << frames << "_" << idx << ".pcd";
		pcl::io::loadPCDFile<PointType>(ss.str(), *cluster);

		pcl::visualization::PCLVisualizer viewer2 ("model");

		viewer2.addPointCloud(cluster, "model");
		viewer2.addCoordinateSystem(0.1, 0);
		viewer2.setPosition(600, 0);

		while (!viewer2.wasStopped())
		{
			viewer2.spinOnce();
		}
	}

	void SetOutputDirectory(string outdir)
	{
		OUT_DIR = outdir;
	}

	void SetFrameNumber(int framenum)
	{
		frame_number = framenum;
	}

	pcl::visualization::CloudViewer viewer;
private:
	int frames_saved;
	bool save_model;
	bool train_model;
	string OUT_DIR;
	int ncluster;

	int frame_number;
};


int main(int argc,char **argv)
{ 
	ros::init(argc, argv, "RS_TrainMain");
    ros::NodeHandle n;
	SimpleRealSenseViewer v;
    ros::Subscriber cloud_sub = n.subscribe("/camera/depth/points", 1, &SimpleRealSenseViewer::cloud_cb,&v);
    boost::thread* thr = new boost::thread(boost::bind(&SimpleRealSenseViewer::run, &v));
    v.SetInputDirectory("/home/iclab-ming/3d_ws/devel/lib/CG_Online/");
    ros::spin();
    v.run();
    ros::Rate loop_rate(100);
    
}
