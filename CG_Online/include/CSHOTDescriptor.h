#include "shot.h"
#include "utils.h"
#include <iostream>
#include <vector>
#include <time.h>
//#include <Windows.h>

#include <fstream>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/uniform_sampling.h>


// Type
typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalType;

class CSHOTDescriptor
{
public:
	CSHOTDescriptor()
	{
		model_ss_ = 0.01f;
		shotShapeBins = 10;
		shotColorBins = 30;
		minNeighbors = 5;
		describeShape = true;
		describeColor = true;
		nThreads = 0;

		nActualFeat = 0;

		model = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT> ());
		normals = pcl::PointCloud<NormalType>::Ptr (new pcl::PointCloud<NormalType> ());
		model_normals = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());

		feat = new Feature3D[10000];
		mesh = vtkPolyData::New();
	}	

	void PCL2VTK()
	{
		pcl::concatenateFields(*model, *normals, *model_normals);

		pcl::io::pointCloudTovtkPolyData(*model_normals, mesh);
	}

	void DescriptorExtract(pcl::PointCloud<pcl::SHOT1344>::Ptr &descriptors)
	{		
		PCL2VTK();

		if (mesh)
		{
			SHOTParams shotParams;
			shotParams.radius = 0.06;
			shotParams.localRFradius = 0.06;
			shotParams.minNeighbors = minNeighbors;
			shotParams.shapeBins = shotShapeBins;
			shotParams.colorBins = shotColorBins;
			shotParams.describeColor = describeColor;
			shotParams.describeShape = describeShape;
			shotParams.nThreads = nThreads;

			SHOTDescriptor descriptor(shotParams);

			double** desc;
			descriptor.describe(mesh, feat, desc, nActualFeat);

			for (int i = 0; i < nActualFeat; i++)
			{
				for (int j = 0; j < descriptor.getDescriptorLength(); j++)
				{
					descriptors->points[i].descriptor[j] = desc[i][j];
				}

				for (int k = 0; k < 9; k++)
				{
					descriptors->points[i].rf[k] = feat[i].rf[k];
				}
			}
		}
	}

	void setInputCloud(pcl::PointCloud<PointType>::Ptr &cloud, pcl::PointCloud<int> &sampled_indices)
	{
		// Set search surface
		pcl::copyPointCloud(*cloud, *model);

		// Set keypoints
		for (int idx = 0; idx < sampled_indices.size(); idx++)
		{
			feat[idx].x = model->points[sampled_indices.points[idx]].x;
			feat[idx].y = model->points[sampled_indices.points[idx]].y;
			feat[idx].z = model->points[sampled_indices.points[idx]].z;

			feat[idx].scale = -1;
			feat[idx].index = sampled_indices.points[idx];
		}

		nActualFeat = sampled_indices.size();
	}

	void setInputNormal(pcl::PointCloud<NormalType>::Ptr cloud_normals)
	{
		// Set point normals
		//pcl::copyPointCloud(*cloud_normals, *normals);
		normals = cloud_normals;
	}


private:
	int minNeighbors;

	int nThreads;

	bool describeColor;
	bool describeShape;

	int shotShapeBins;
	int shotColorBins;

	float model_ss_;

	int nActualFeat;

	pcl::PointCloud<PointT>::Ptr model;
	pcl::PointCloud<NormalType>::Ptr normals;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model_normals;

	Feature3D* feat;
	vtkPolyData* mesh;
};
