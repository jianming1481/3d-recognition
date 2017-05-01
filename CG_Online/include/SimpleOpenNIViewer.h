#pragma once

#include "Recongnition3D.h"
#include <iostream>
#include <string>
#include <sstream>
using namespace std;

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>


class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer("PCL OpenNI Viewer")
    {
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
		v->registerKeyboardCallback<SimpleOpenNIViewer>(&SimpleOpenNIViewer::keyboardEventOccurred, *this); 

		return(v); 
    }

	/*void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
		pcl::PCLPointCloud2 pcl_pc2;
		pcl_conversions::toPCL(*input,pcl_pc2);
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
		//do stuff with temp_cloud here
	}*/

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
		if (!viewer.wasStopped()) 
		{
			viewer.showCloud (cloud);

			if ( start_cg ) 
			{
				
				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr copycloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

				if (frames_saved == 1)
				{
					int id = 0;
					cout << "Input object ID:\n";
					cin >> id;
					LoadPCDFile(model, id);
					LoadTXTData(model_descriptors, model_keypoints, id);
				}
				
				pcl::copyPointCloud(*cloud, *copycloud);

				Recongnition3D recg3d;
				recg3d.InputCloud(model, copycloud);
				recg3d.InputFeature(model_descriptors, model_keypoints);
				recg3d.RecongnitionPipeline();			
			
				start_cg = false;
			}
			
		}
    }

    void run ()
	{
		pcl::Grabber* pclif = new pcl::OpenNIGrabber();

		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
			boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

		pclif->registerCallback (f);

		pclif->start ();

		char c;
		char data[2];
		long size = 0;
		int i = 1;

		while (!viewer.wasStopped())
        { 
			//sleep (1); 
			boost::this_thread::sleep (boost::posix_time::seconds (1)); 

			c = getchar();
			if ( c == 'r' )
			{
				frames_saved++;
				cout << "frame " << frames_saved << ".\n";
				start_cg = true;			
				sprintf(data, "%d", frames_saved);
			}
			

		}
		
        pclif->stop (); 
	}

	void SetInputDirectory(string outdir)
	{
		OUT_DIR = outdir;
	}

	void SetFrameNumber(int framenum)
	{
		frame_number = framenum;
	}

	void LoadPCDFile(pcl::PointCloud<PointType>::Ptr &model, int idx)
	{
		std::stringstream pcdpath;

		pcdpath << OUT_DIR.c_str() << "cloud_cluster_1_" << idx << ".pcd";

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