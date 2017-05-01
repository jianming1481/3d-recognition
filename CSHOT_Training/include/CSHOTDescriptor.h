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

		model = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT> ());
		normals = pcl::PointCloud<NormalType>::Ptr (new pcl::PointCloud<NormalType> ());
		model_normals = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());

		feat = new Feature3D[1000];
		mesh = vtkPolyData::New();
	}

	

	//  Downsample clouds to extract keypoints.
	int DownsampleCload()
	{
		pcl::PointCloud<PointT>::Ptr keypoints(new pcl::PointCloud<PointT> ());
		pcl::PointCloud<int> sampled_indices;

		pcl::UniformSampling<PointT> uniform_sampling;
		uniform_sampling.setInputCloud (model);
		uniform_sampling.setRadiusSearch (model_ss_);
		uniform_sampling.compute (sampled_indices);
		pcl::copyPointCloud (*model, sampled_indices.points, *keypoints);

		for (int idx = 0; idx < sampled_indices.size(); idx++)
		{
			feat[idx].x = model->points[sampled_indices.points[idx]].x;
			feat[idx].y = model->points[sampled_indices.points[idx]].y;
			feat[idx].z = model->points[sampled_indices.points[idx]].z;

			feat[idx].scale = -1;
			feat[idx].index = sampled_indices.points[idx];
		}

		return ( sampled_indices.size() );
	}

	void PCL2VTK()
	{
		//  Compute point normals
		pcl::NormalEstimationOMP<PointT, NormalType> norm_est;
		norm_est.setKSearch (10);
		norm_est.setInputCloud (model);
		norm_est.compute (*normals);

		pcl::concatenateFields(*model, *normals, *model_normals);

		pcl::io::pointCloudTovtkPolyData(*model_normals, mesh);
	}

	void DescriptorExtract(std::string &FILE_PATH, int frame, int idx)
	{
		std::stringstream ss;

		ss << FILE_PATH.c_str() << "cloud_cluster_" << frame << "_" << idx << ".pcd";

		if (pcl::io::loadPCDFile<PointT>(ss.str(), *model) < 0)
		{
			PCL_INFO("Error loading model cloud.");
		}
		
		PCL2VTK();

		if (mesh)
		{
			int nActualFeat = DownsampleCload();

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

			std::stringstream out;

			out << FILE_PATH.c_str() << "cshot_" << frame << "_" << idx << ".txt";

			std::ofstream outfile(out.str());
			if (!outfile.is_open())
				std::cout << "\nWARNING\n It is not possible to write on the requested output file\n";

			if (outfile.is_open())
				outfile << nActualFeat << "\n";

			for (int i = 0; i < nActualFeat; i++)
			{
				if (outfile.is_open())
					outfile << feat[i].index;

				for (int k = 0; k < 9; k++)
				{
					if (outfile.is_open())
						outfile << " " << feat[i].rf[k];
				}

				for (int j = 0; j < descriptor.getDescriptorLength(); j++)
				{
					if (outfile.is_open())
						outfile << " " << desc[i][j];
				}
				if (outfile.is_open())
					outfile << "\n ";
			}
		}
	}


private:
	int minNeighbors;

	int nThreads;

	bool describeColor;
	bool describeShape;

	int shotShapeBins;
	int shotColorBins;

	float model_ss_;

	pcl::PointCloud<PointT>::Ptr model;
	pcl::PointCloud<NormalType>::Ptr normals;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model_normals;

	Feature3D* feat;
	vtkPolyData* mesh;
};
