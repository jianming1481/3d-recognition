#include "OrganizedSegmentation.h"
#include "CSHOTDescriptor.h"

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/features/integral_image_normal.h>
//#include <pcl/filters/uniform_sampling.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/console/parse.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/sample_consensus/sac_model_registration.h>

typedef pcl::PointXYZRGBA PointType;
//typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT1344 DescriptorType;
typedef pcl::SHOT352 SHOT352Type;
//struct timeval tpstart,tpend;

int v1(0);
int v2(0);
//count time
#define count_time

class Recongnition3D
{
public:
	Recongnition3D()
	{
		model = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType> ());
		scene = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType> ());
		model_keypoints = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType> ());
		scene_keypoints = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType> ());
		scene_1344keypoints = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType> ());

		ori_scene = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType> ());

		sampled_indices.reserve(1000);

		model_normals = pcl::PointCloud<NormalType>::Ptr (new pcl::PointCloud<NormalType> ());
		scene_normals = pcl::PointCloud<NormalType>::Ptr (new pcl::PointCloud<NormalType> ());
		model_descriptors = pcl::PointCloud<DescriptorType>::Ptr (new pcl::PointCloud<DescriptorType> ());
		scene_descriptors = pcl::PointCloud<DescriptorType>::Ptr (new pcl::PointCloud<DescriptorType> ());
		scene_1344descriptors = pcl::PointCloud<DescriptorType>::Ptr (new pcl::PointCloud<DescriptorType> ());
		shape_corrs = pcl::CorrespondencesPtr (new pcl::Correspondences ());
		model_scene_corrs = pcl::CorrespondencesPtr (new pcl::Correspondences ());

		viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer ("Result"));
		viewer->setPosition(600, 0);
		viewer2 = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer ("Model"));
		viewer2->setPosition(700, 400);

		//use_UniformSampling_ = false;
		show_keypoints_ = false;
		show_correspondences_ = false;
		use_hough_ = true;

		//Algorithm params
		model_ss_ = 0.01f;
		scene_ss_ = 0.015f;
		rf_rad_ = 0.06f;
		descr_rad_ = 0.06f;
		cg_size_ = 0.05f;
		cg_thresh_ = 3.0f;

		//ICP params
		int icp_max_iter_ = 5;
		float icp_corr_distance_ = 0.005f;

		//GO Hypotheses Verification params
		hv_clutter_reg_ = 2.0f;
		hv_inlier_th_ = 0.0075f;
		hv_occlusion_th_ = 0.01f;
		hv_rad_clutter_ = 0.03f;
		hv_regularizer_ = 5.0f;
		hv_rad_normals_ = 0.05f;
		hv_detect_clutter_ = true;

		rotatematrix.resize(10);
		fout = fopen("pose_data.txt", "a+");
	}

	//  Compute Normals
	void ComputeNormals()
	{
#ifdef count_time
		typedef LARGE_INTEGER nFreq;
		typedef LARGE_INTEGER nBeginTime;
		typedef LARGE_INTEGER nEndTime;
		double time1;
#endif

#ifdef count_time
		//QueryPerformanceFrequency(&nFreq);
		//QueryPerformanceCounter(&nBeginTime);
		gettimeofday(&tpstart,NULL);
#endif

		pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
		norm_est.setKSearch (10);
		norm_est.setInputCloud (model);
		norm_est.compute (*model_normals);

		norm_est.setInputCloud (scene);
		norm_est.compute (*scene_normals);

#ifdef count_time
		//QueryPerformanceCounter(&nEndTime);
		//time1=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart;
		gettimeofday(&tpend,NULL);
		time1=1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec;
		cout << "Compute normals time: " << time1 << endl;
		cout << "-------------------------------" << endl;
#endif
	}

	//  Downsample Clouds to Extract keypoints
	void DownsampleCload()
	{
#ifdef count_time
		LARGE_INTEGER nFreq;
		LARGE_INTEGER nBeginTime;
		LARGE_INTEGER nEndTime;
		double time1;
#endif

#ifdef count_time
		//QueryPerformanceFrequency(&nFreq);
		//QueryPerformanceCounter(&nBeginTime);
		gettimeofday(&tpstart,NULL);
#endif

		pcl::UniformSampling<PointType> uniform_sampling;

		uniform_sampling.setInputCloud (scene);
		uniform_sampling.setRadiusSearch (scene_ss_);
		//uniform_sampling.filter (*scene_1344keypoints);
		uniform_sampling.compute (sampled_indices);
		
		pcl::copyPointCloud (*scene, sampled_indices.points, *scene_1344keypoints);
		std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_1344keypoints->size () << std::endl;

#ifdef count_time
		//QueryPerformanceCounter(&nEndTime);
		//time1=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart;
		gettimeofday(&tpend,NULL);
		time1=1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec;
		cout << "Extract keypoints time: " << time1 << endl;
		cout << "-------------------------------" << endl;
#endif
	}

	void ExtractDescriptor()
	{
#ifdef count_time
		LARGE_INTEGER nFreq;
		LARGE_INTEGER nBeginTime;
		LARGE_INTEGER nEndTime;
		double time1;
#endif

#ifdef count_time
		//QueryPerformanceFrequency(&nFreq);
		//QueryPerformanceCounter(&nBeginTime);
		gettimeofday(&tpstart,NULL);
#endif

		scene_1344descriptors->resize(sampled_indices.size());

		CSHOTDescriptor cshot;
		cshot.setInputCloud(scene, sampled_indices);
		cshot.setInputNormal(scene_normals);
		cshot.DescriptorExtract(scene_1344descriptors);

		/*pcl::SHOTColorEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
		descr_est.setRadiusSearch (descr_rad_);

		descr_est.setInputCloud (scene_1344keypoints);
		descr_est.setInputNormals (scene_normals);
		descr_est.setSearchSurface (scene);
		descr_est.compute (*scene_1344descriptors);*/

#ifdef count_time
		//QueryPerformanceCounter(&nEndTime);
		//time1=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart;
		gettimeofday(&tpend,NULL);
		time1=1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec;
		cout << "Extract descriptors time: " << time1 << endl;
		cout << "-------------------------------" << endl;
#endif
	}

	void DescriptorMatching()
	{
#ifdef count_time
		LARGE_INTEGER nFreq;
		LARGE_INTEGER nBeginTime;
		LARGE_INTEGER nEndTime;
		double time1;
#endif

#ifdef count_time
		//QueryPerformanceFrequency(&nFreq);
		//QueryPerformanceCounter(&nBeginTime);
		gettimeofday(&tpstart,NULL);
#endif

		pcl::PointCloud<SHOT352Type>::Ptr model_352descriptors (new pcl::PointCloud<SHOT352Type> ());
		pcl::PointCloud<SHOT352Type>::Ptr scene_352descriptors (new pcl::PointCloud<SHOT352Type> ());

		model_352descriptors->resize(model_descriptors->size());
		scene_352descriptors->resize(scene_1344descriptors->size());

		for (int i = 0; i < model_352descriptors->size(); i++)
		{
			for (int j = 0; j < 352; j++)
			{
				model_352descriptors->points[i].descriptor[j] = model_descriptors->points[i].descriptor[j];
			}

			for (int k = 0; k < 9; k++)
			{
				model_352descriptors->points[i].rf[k] = model_descriptors->points[i].rf[k];
			}
		}

		for (int i = 0; i < scene_352descriptors->size(); i++)
		{
			for (int j = 0; j < 352; j++)
			{
				scene_352descriptors->points[i].descriptor[j] = scene_1344descriptors->points[i].descriptor[j];
			}

			for (int k = 0; k < 9; k++)
			{
				scene_352descriptors->points[i].rf[k] = scene_1344descriptors->points[i].rf[k];
			}
		}

		std::vector<int> query_idx;
		std::vector<int> match_idx;

		// First matching
		pcl::KdTreeFLANN<SHOT352Type> matchshape;
		matchshape.setInputCloud(model_352descriptors);
		//matchshape.setEpsilon(0.05);

		for (size_t i = 0; i < scene_352descriptors->size (); ++i)
		{
			std::vector<int> neigh_indices (1);
			std::vector<float> neigh_sqr_dists (1);
			if (!pcl_isfinite (scene_352descriptors->at (i).descriptor[0])) //skipping NaNs
			{
				continue;
			}
			int found_neighs = matchshape.nearestKSearch (scene_352descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
			if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
			{
				//pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
				//shape_corrs->push_back (corr);

				match_idx.push_back(static_cast<int> (i));
			}
		}

		std::cout << "First correspondences found: " << match_idx.size () << std::endl;
		
		
		pcl::copyPointCloud(*scene_1344keypoints, match_idx, *scene_keypoints);
		pcl::copyPointCloud(*scene_1344descriptors, match_idx, *scene_descriptors);
		
		// Second matching
		pcl::KdTreeFLANN<DescriptorType> match_search;
		match_search.setInputCloud (model_descriptors);
		//match_search.setEpsilon(0.0);

		//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
		for (size_t i = 0; i < scene_descriptors->size (); ++i)
		{
			std::vector<int> neigh_indices (1);
			std::vector<float> neigh_sqr_dists (1);
			if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
			{
				continue;
			}
			int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
			if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
			{
				pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
				model_scene_corrs->push_back (corr);
			}
		}

		std::cout << "Second correspondences found: " << model_scene_corrs->size () << std::endl;

#ifdef count_time
		//QueryPerformanceCounter(&nEndTime);
		//time1=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart;
		gettimeofday(&tpend,NULL);
		time1=1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec;
		cout << "Matching time: " << time1 << endl;
		cout << "-------------------------------" << endl;
#endif
	}

	//  Actual Clustering(Correspondence grouping)
	void Correspondence_Grouping()
	{
#ifdef count_time
		LARGE_INTEGER nFreq;
		LARGE_INTEGER nBeginTime;
		LARGE_INTEGER nEndTime;
		double time1;
#endif

#ifdef count_time
		//QueryPerformanceFrequency(&nFreq);
		//QueryPerformanceCounter(&nBeginTime);
		gettimeofday(&tpstart,NULL);
#endif

		//  Using Hough3D
		if (use_hough_)
		{
			//
			//  Compute (Keypoints) Reference Frames only for Hough
			//
			pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
			pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());


			pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
			rf_est.setFindHoles (true);
			rf_est.setRadiusSearch (rf_rad_);

			rf_est.setInputCloud (model_keypoints);
			rf_est.setInputNormals (model_normals);
			rf_est.setSearchSurface (model);
			rf_est.compute (*model_rf);

			rf_est.setInputCloud (scene_keypoints);
			rf_est.setInputNormals (scene_normals);
			rf_est.setSearchSurface (scene);
			rf_est.compute (*scene_rf);

			//  Clustering
			pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
			clusterer.setHoughBinSize (cg_size_);
			clusterer.setHoughThreshold (cg_thresh_);
			clusterer.setUseInterpolation (true);
			clusterer.setUseDistanceWeight (false);

			clusterer.setInputCloud (model_keypoints);
			clusterer.setInputRf (model_rf);
			clusterer.setSceneCloud (scene_keypoints);
			clusterer.setSceneRf (scene_rf);
			clusterer.setModelSceneCorrespondences (model_scene_corrs);

			//clusterer.cluster (clustered_corrs);
			clusterer.recognize (rototranslations, clustered_corrs);

		}
		else // Using Geometric Consistency
		{
			pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
			gc_clusterer.setGCSize (cg_size_);
			gc_clusterer.setGCThreshold (cg_thresh_);

			gc_clusterer.setInputCloud (model_keypoints);
			gc_clusterer.setSceneCloud (scene_keypoints);
			gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

			//gc_clusterer.cluster (clustered_corrs);
			gc_clusterer.recognize (rototranslations, clustered_corrs);

		}

#ifdef count_time
		//QueryPerformanceCounter(&nEndTime);
		//time1=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart;
		gettimeofday(&tpend,NULL);
		time1=1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec;
		cout << "correspondence grouping time: " << time1 << endl;
		cout << "-------------------------------" << endl;
#endif

		//  Output correspondence grouping results
		std::cout << "--correspondence grouping result--" << std::endl;

		std::cout << "Model instances found: " << rototranslations.size () << std::endl;
		for (size_t i = 0; i < rototranslations.size (); ++i)
		{
			std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
			std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

			// Print the rotation matrix and translation vector
			Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
			Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

			printf ("\n");
			printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
			printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
			printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
			printf ("\n");
			printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
			printf ("Degree: < %f, %f, %f >\n", atan2(rotation (2,1), rotation (2,2))*180.0/M_PI, 
				atan2(-rotation (2,0), sqrt(rotation (2,1)*rotation (2,1) + rotation (2,2)*rotation (2,2)))*180.0/M_PI,
				atan2(rotation (1,0), rotation (0,0))*180.0/M_PI);
			std::cout << "------------------------------" << std::endl;
		}
	}

	//
	//  Absolute Orientation
	//
	void AbsoluteOrientation()
	{
#ifdef count_time
		LARGE_INTEGER nFreq;
		LARGE_INTEGER nBeginTime;
		LARGE_INTEGER nEndTime;
		double time1;
#endif

#ifdef count_time
		//QueryPerformanceFrequency(&nFreq);
		//QueryPerformanceCounter(&nBeginTime);
		gettimeofday(&tpstart,NULL);
#endif

		std::vector<pcl::PointCloud<PointType>::ConstPtr> keypoint_instances;
		//  Generates clouds for each instances found
		for (int i = 0; i < rototranslations.size(); i++)
		{
			/*pcl::PointCloud<PointType>::Ptr RotateModel(new pcl::PointCloud<PointType>);
			pcl::transformPointCloud(*model, *RotateModel, rototranslations[i]);
			instances.push_back(RotateModel);*/

			pcl::PointCloud<PointType>::Ptr RotateModelKeypoint(new pcl::PointCloud<PointType>);
			pcl::transformPointCloud(*model_keypoints, *RotateModelKeypoint, rototranslations[i]);
			keypoint_instances.push_back(RotateModelKeypoint);
		}

		std::cout << "------------RANSAC-------------" << std::endl;

		for (int i = 0; i < rototranslations.size(); i++)
		{
			// Object for pose estimation.
			pcl::SampleConsensusPrerejective<PointType, PointType, DescriptorType> ransac;
			ransac.setInputSource(keypoint_instances[i]);
			ransac.setInputTarget(scene_keypoints);
			ransac.setSourceFeatures(model_descriptors);
			ransac.setTargetFeatures(scene_descriptors);
			// Instead of matching a descriptor with its nearest neighbor, choose randomly between
			// the N closest ones, making it more robust to outliers, but increasing time.
			ransac.setCorrespondenceRandomness(2);
			// Set the fraction (0-1) of inlier points required for accepting a transformation.
			// At least this number of points will need to be aligned to accept a pose.
			ransac.setInlierFraction(0.25f);
			// Set the number of samples to use during each iteration (minimum for 6 DoF is 3).
			ransac.setNumberOfSamples(3);
			// Set the similarity threshold (0-1) between edge lengths of the polygons. The
			// closer to 1, the more strict the rejector will be, probably discarding acceptable poses.
			ransac.setSimilarityThreshold(0.6f);
			// Set the maximum distance threshold between two correspondent points in source and target.
			// If the distance is larger, the points will be ignored in the alignment process.
			ransac.setMaxCorrespondenceDistance(0.005f);

			pcl::PointCloud<PointType>::Ptr registered(new pcl::PointCloud<PointType>);
			ransac.align(*registered);
			//registered_instances.push_back(registered);

			if (ransac.hasConverged())
			{
				Eigen::Matrix4f transformation = ransac.getFinalTransformation();

				rototranslations[i] = transformation * rototranslations[i];

				Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
				Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);

				std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
				std::cout << "Transformation matrix:" << std::endl << std::endl;
				printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
				printf("\t\tR = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
				printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
				std::cout << std::endl;
				//printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
				printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", transformation(0,3)+rototranslations[i](0,3), 
					transformation(1,3)+rototranslations[i](1,3), 
					transformation(2,3)+rototranslations[i](2,3));
				printf ("Degree: < %f, %f, %f >\n", atan2(rotation (2,1), rotation (2,2))*180.0/M_PI, 
					atan2(-rotation (2,0), sqrt(rotation (2,1)*rotation (2,1) + rotation (2,2)*rotation (2,2)))*180.0/M_PI,
					atan2(rotation (1,0), rotation (0,0))*180.0/M_PI);

				rotatematrix[i] = rotation;
			}
			else
			{ 
				std::cout << "Did not converge." << std::endl;
			}
		}		
		std::cout << "------------------------------" << std::endl;

#ifdef count_time
		//QueryPerformanceCounter(&nEndTime);
		//time1=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart;
		gettimeofday(&tpend,NULL);
		time1=1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec;
		cout << "Absolute orientation time: " << time1 << endl;
		cout << "-------------------------------" << endl;
#endif
	}

	//
	//  ICP Refinement
	//
	void ICPRefinement()
	{
		//  Generates clouds for each instances found
		for (int i = 0; i < rototranslations.size(); i++)
		{
			pcl::PointCloud<PointType>::Ptr RotateModel(new pcl::PointCloud<PointType>);
			pcl::transformPointCloud(*model, *RotateModel, rototranslations[i]);
			instances.push_back(RotateModel);
		}

		//  ICP*********08/19*********
		std::cout << "------------ICP---------------" << std::endl;

		for (int i = 0; i < rototranslations.size(); i++)
		{
			pcl::IterativeClosestPoint<PointType, PointType> icp;
			icp.setMaximumIterations (icp_max_iter_);
			icp.setMaxCorrespondenceDistance (icp_corr_distance_);
			icp.setInputTarget (scene);
			icp.setInputSource (instances[i]);
			pcl::PointCloud<PointType>::Ptr registered(new pcl::PointCloud<PointType>);
			icp.align(*registered);
			registered_instances.push_back(registered);
			if (icp.hasConverged())
			{
				// Output ICP results
				std::cout << "Instance" << i + 1 << " is converged." << std::endl
					<< "The score is " << icp.getFitnessScore() << std::endl;
				std::cout << "Transformation matrix:" << std::endl;
				std::cout << icp.getFinalTransformation() << std::endl;
				if(acos(icp.getFinalTransformation() (0,0)) < 0.000001)
				{
					printf ("Degree: %f\n", acos(icp.getFinalTransformation() (0,0))*180.0/M_PI);
				}
				else
				{
					printf ("Degree: %f\n", 0.0);
				}
			} 
			else
			{
				std::cout << "Instance" << i + 1 << " is Not converged" << std::endl;
			}
		}
		std::cout << "------------------------------" << std::endl;
	}

	//	Hypothesis Verification
	void GOHypothesisVerification()
	{
		std::cout << "---Hypotheses Verification----" << std::endl;
		std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

		pcl::GlobalHypothesesVerification<PointType, PointType> GoHv;

		GoHv.setSceneCloud (scene);  // Scene Cloud
		GoHv.addModels (registered_instances, true);  //Models to verify

		GoHv.setInlierThreshold (hv_inlier_th_);
		GoHv.setOcclusionThreshold (hv_occlusion_th_);
		GoHv.setRegularizer (hv_regularizer_);
		GoHv.setRadiusClutter (hv_rad_clutter_);
		GoHv.setClutterRegularizer (hv_clutter_reg_);
		GoHv.setDetectClutter (hv_detect_clutter_);
		GoHv.setRadiusNormals (hv_rad_normals_);

		GoHv.verify ();
		GoHv.getMask (hypotheses_mask);  // i-element TRUE if hvModels[i] verifies hypotheses

		for (int i = 0; i < hypotheses_mask.size (); i++)
		{
			if (hypotheses_mask[i])
			{
				cout << "Instance " << i + 1 << " is Good! <---" << endl;
			}
			else
			{
				cout << "Instance " << i + 1 << " is Bad!" << endl;
			}
		}
		cout << "-------------------------------" << endl;

		//	Registered viewer
		for (size_t i = 0; i < instances.size (); ++i)
		{
			std::stringstream ss_instance;
			std::stringstream ss;
			/*ss_instance << "instance_" << i;

			pcl::visualization::PointCloudColorHandlerCustom<PointType> instance_color_handler (instances[i], 255, 0, 0);
			viewer.addPointCloud (instances[i], instance_color_handler, ss_instance.str ());
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss_instance.str ());*/

			if (hypotheses_mask[i])
			{
				ss_instance << "_registered" << i << endl;
				viewer->createViewPort (0, 0, 0.5, 1.0, v1);
				viewer->addPointCloud (ori_scene, "scene", v1);
				viewer->createViewPort (0.5, 0, 1.0, 1.0, v2);
				viewer->addPointCloud (ori_scene, "result", v2);	
				pcl::visualization::PointCloudColorHandlerCustom<PointType> registered_instance_color_handler (registered_instances[i], 0,
					255, 0);
				viewer->addPointCloud (registered_instances[i], registered_instance_color_handler, ss_instance.str (), v2);
				viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, ss_instance.str ());
				viewer->addCoordinateSystem(0.1, 0);

				ss << "Instance_" << i + 1;
				Eigen::Vector4f centroid;
				pcl::compute3DCentroid (*registered_instances[i], centroid);
				printf("center point = < %6.3f, %6.3f, %6.3f >\n", centroid(0)*100, centroid(1)*100, centroid(2)*100);
				PointType point;
				point.getArray4fMap() << centroid;
				//viewer->addText3D(ss.str (), point, 0.02, 1.0, 1.0, 0.0, ss.str (), v2);
				ss.str("");
				ss.clear();

				fprintf(fout, "%6.3f\t%6.3f\t%6.3f\t%6.3f\t%6.3f\t%6.3f\n", centroid(0)*100, centroid(1)*100, centroid(2)*100, 
					atan2(rotatematrix[i] (2,1), rotatematrix[i] (2,2))*180.0/M_PI, 
					atan2(-rotatematrix[i] (2,0), sqrt(rotatematrix[i] (2,1)*rotatematrix[i] (2,1) + rotatematrix[i] (2,2)*rotatematrix[i] (2,2)))*180.0/M_PI,
					atan2(rotatematrix[i] (1,0), rotatematrix[i] (0,0))*180.0/M_PI);
				fclose(fout);
			} 
			else
			{
				/*ss_instance << "_registered" << i << endl;
				pcl::visualization::PointCloudColorHandlerCustom<PointType> registered_instance_color_handler (registered_instances[i], 255,
					0, 0);
				viewer->addPointCloud (registered_instances[i], registered_instance_color_handler, ss_instance.str ());
				viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, ss_instance.str ());*/
			}
		}
	}

	//  Visualization
	void PCLVisualization()
	{	
		viewer2->addPointCloud (model, "model_cloud");
		viewer2->addCoordinateSystem(0.1, 0);		
		//viewer2->setBackgroundColor(255.0, 255.0, 255.0);

		pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
		pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

		if (show_correspondences_ || show_keypoints_)
		{
			//  We are translating the model so that it doesn't end in the middle of the scene representation
			pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
			pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

			pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
			viewer->addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
			viewer->addCoordinateSystem(0.1, 0);
		}

		if (show_keypoints_)
		{
			pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
			viewer->addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

			pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
			viewer->addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
		}

		for (size_t i = 0; i < rototranslations.size (); ++i)
		{
			pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
			pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

			std::stringstream ss_cloud;
			ss_cloud << "instance" << i;

			/*pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
			viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());*/

			if (show_correspondences_)
			{
				for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
				{
					std::stringstream ss_line;
					ss_line << "correspondence_line" << i << "_" << j;
					PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
					PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

					//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
					viewer->addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
				}
			}
		}

		while (!viewer->wasStopped ())
		{
			viewer->spinOnce ();
		}
	}

	void RecongnitionPipeline()
	{		
#ifdef count_time
		LARGE_INTEGER nFreq;
		LARGE_INTEGER nBeginTime;
		LARGE_INTEGER nEndTime;
		double time2;
#endif

		if (scene->size() > 0)
		{

#ifdef count_time
			//QueryPerformanceFrequency(&nFreq);
			//QueryPerformanceCounter(&nBeginTime);
			gettimeofday(&tpstart,NULL);
#endif

			// model&scene
			ComputeNormals();
		
			// only scene	
			DownsampleCload();

			ExtractDescriptor();

			// Matching
			DescriptorMatching();

			// Recongnition and pose estimation.
			Correspondence_Grouping();
			if (model_scene_corrs->size () >= 30)
			{
				AbsoluteOrientation();
			}
#if 0
			ICPRefinement();
			if (rototranslations.size ())
			{
				GOHypothesisVerification();
			}

#ifdef count_time
			//QueryPerformanceCounter(&nEndTime);
			//time2=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart;
			gettimeofday(&tpend,NULL);
			time2=1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec;
			cout << "Recognition + Pose estimation time: " << time2 << endl;
			cout << "-------------------------------" << endl;
#endif
#endif
			// show result
			PCLVisualization();
		}
	}

	void InputCloud(pcl::PointCloud<PointType>::Ptr &MODEL, pcl::PointCloud<PointType>::Ptr &SCENE)
	{
		// Have a problem !!!
		/*Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

		transformation(2, 3) = 0.8;

		pcl::transformPointCloud(*MODEL, *model, transformation);*/

		//pcl::copyPointCloud(*MODEL, *model);
		model = MODEL;
		ori_scene = SCENE;

		segment.FindSegmentPlaneScene(SCENE, scene);
		//scene = MODEL;  //test.
	}   

	void InputFeature(pcl::PointCloud<DescriptorType>::Ptr &descriptors, pcl::PointCloud<PointType>::Ptr &keypoints)
	{
		model_descriptors = descriptors;

		model_keypoints = keypoints;
	}


private:
	pcl::PointCloud<PointType>::Ptr model;
	pcl::PointCloud<PointType>::Ptr scene;
	pcl::PointCloud<PointType>::Ptr model_keypoints;
	pcl::PointCloud<PointType>::Ptr scene_keypoints;
	pcl::PointCloud<PointType>::Ptr scene_1344keypoints;

	pcl::PointCloud<PointType>::Ptr ori_scene;

	pcl::PointCloud<int> sampled_indices;

	pcl::PointCloud<NormalType>::Ptr model_normals;
	pcl::PointCloud<NormalType>::Ptr scene_normals;
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors;
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors;
	pcl::PointCloud<DescriptorType>::Ptr scene_1344descriptors;
	pcl::CorrespondencesPtr shape_corrs;
	pcl::CorrespondencesPtr model_scene_corrs;

	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;
	std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;
	std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;

	//bool use_UniformSampling_;
	bool use_hough_;
	bool show_keypoints_;
	bool show_correspondences_;

	//Algorithm params
	float model_ss_;
	float scene_ss_;
	float rf_rad_;
	float descr_rad_;
	float cg_size_;
	float cg_thresh_;

	//ICP params
	int icp_max_iter_;
	float icp_corr_distance_;

	//GO Hypotheses Verification params
	float hv_clutter_reg_;
	float hv_inlier_th_;
	float hv_occlusion_th_;
	float hv_rad_clutter_;
	float hv_regularizer_;
	float hv_rad_normals_;
	bool hv_detect_clutter_;

	//Class
	OrganizedSegmentation segment;

	std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > rotatematrix;
	FILE *fout;
};