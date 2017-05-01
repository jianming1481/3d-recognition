#pragma once

#include "OrganizedSegmentation.h"
#include "CSHOTDescriptor.h"

#include <iostream>
#include <string>
#include <sstream>
using namespace std;

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#define SAVE_FULL_CLOUD

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer("PCL OpenNI Viewer")
    {
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
		v->registerKeyboardCallback<SimpleOpenNIViewer>(&SimpleOpenNIViewer::keyboardEventOccurred, *this); 

		return(v); 
    }

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
		if (!viewer.wasStopped()) 
		{
			viewer.showCloud (cloud);

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
				ss1 << "cloud" << frames_saved << ".pcd";
				writer1.write<PointType> (ss1.str (), *cloud, false);
				ss1.str("");
				ss1.clear();
#endif
				
				save_model = false;
			}
			else if ( train_model )
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
			boost::this_thread::sleep (boost::posix_time::seconds (2)); 

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

		}
		
        pclif->stop (); 
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