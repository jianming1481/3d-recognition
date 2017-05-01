#pragma once

#include "../include/Recongnition3D.h"
#include "ros/ros.h"
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


class SimpleRealSenseViewer
{
public:
    SimpleRealSenseViewer() : viewer("Cloud Viewer")
    {
        ROS_INFO("Initialize SimeleRealSenseViewer Class");
        model = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType> ());
		model_keypoints = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType> ());
		model_descriptors = pcl::PointCloud<DescriptorType>::Ptr (new pcl::PointCloud<DescriptorType> ());

        frames_saved = 0;
        start_cg = false;
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
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
        if (!viewer.wasStopped()) 
        {
            viewer.showCloud(temp_cloud);
            if ( start_cg ) 
			{
				
				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr copycloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

				if (frames_saved == 1)
				{
					int id = 0;
					cout << "Input object ID:\n";
					cin >> id;
					LoadPCDFile(model,id);
					LoadTXTData(model_descriptors, model_keypoints, id);
				}
				
				pcl::copyPointCloud(*temp_cloud, *copycloud);

				Recongnition3D recg3d;
				recg3d.InputCloud(model, copycloud);
				recg3d.InputFeature(model_descriptors, model_keypoints);
				recg3d.RecongnitionPipeline();			
			
				start_cg = false;
			}
        }
		// run();
	}

    void run ()
	{

		char c;
		char data[2];
		long size = 0;
		int i = 1;

        c = getchar();
        if ( c == 'r' )
        {
            frames_saved++;
            cout << "frame " << frames_saved << ".\n";
            start_cg = true;			
            sprintf(data, "%d", frames_saved);
        }
	}

	void SetInputDirectory(string outdir)
	{
		OUT_DIR = outdir;
		cout << "Directory is " << OUT_DIR << endl;
	}

	void SetFrameNumber(int framenum)
	{
		frame_number = framenum;
	}

	void LoadPCDFile(pcl::PointCloud<PointType>::Ptr &model,int idx)
	{
		std::stringstream pcdpath;

		pcdpath << OUT_DIR.c_str() << "cloud_cluster_1_" << idx << ".pcd";
		cout << "PCD Path = "  << pcdpath.str() << endl;

		if (pcl::io::loadPCDFile<PointType>(pcdpath.str(), *model) < 0)
		{
			std::cout << "Error loading model cloud." << std::endl;
		} 
	}

	void LoadTXTData(pcl::PointCloud<DescriptorType>::Ptr &model_descriptors, pcl::PointCloud<PointType>::Ptr &model_keypoints, int idx)
	{
		int nfeat = 0;
		int featidx = 0;
		std::vector<int> featidex;

		std::stringstream txtpath;
		txtpath << OUT_DIR.c_str() << "cshot_1_" << idx << ".txt";
		cout << "TXT Path = "  << txtpath.str() << endl;
		ifstream fin(txtpath.str());

		fin >> nfeat;

		model_descriptors->resize(nfeat);

		for (int i = 0; i < nfeat; i++)
		{
			fin >> featidx;
			featidex.push_back(featidx);

			for (int j = 0; j < 9; j++)
			{
				fin >> model_descriptors->points[i].rf[j];
			}

			for (int k = 0; k < 1344; k++)
			{
				fin >> model_descriptors->points[i].descriptor[k];
			}
		}

		pcl::copyPointCloud(*model, featidex, *model_keypoints);
	}
	pcl::visualization::CloudViewer viewer;
private:
	pcl::PointCloud<PointType>::Ptr model;
	pcl::PointCloud<PointType>::Ptr model_keypoints;
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors;

	int frames_saved;
	bool start_cg;
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
    v.SetInputDirectory("/home/iclab-giga/catkin_ws/devel/lib/CG_Online/");
    ros::Rate loop_rate(100);
    ros::spin();
}