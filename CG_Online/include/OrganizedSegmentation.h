#include <pcl/common/angles.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>

//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <time.h>

typedef pcl::PointXYZRGBA PointType;
bool use_planar_refinement_ = true;

//count time
#define count_time
typedef unsigned long DWORD, *PDWORD, *LPDWORD;
typedef long LONG, *PLONG, *LPLONG;
typedef signed long long LONGLONG;
struct timeval tpstart,tpend;

typedef union _LARGE_INTEGER {
  struct {
    DWORD LowPart;
    LONG  HighPart;
  };
  struct {
    DWORD LowPart;
    LONG  HighPart;
  } u;
  LONGLONG QuadPart;
} LARGE_INTEGER, *PLARGE_INTEGER;

class OrganizedSegmentation
{
public:
	OrganizedSegmentation()
	{
		labels = pcl::PointCloud<pcl::Label>::Ptr (new pcl::PointCloud<pcl::Label>);
		euclidean_cluster_comparator_ = pcl::EuclideanClusterComparator<PointType, pcl::Normal, pcl::Label>::Ptr 
										(new pcl::EuclideanClusterComparator<PointType, pcl::Normal, pcl::Label>());
	}

	void PlaneSegmentation(pcl::PointCloud<PointType>::Ptr &cloud)
	{
		pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);

		std::vector<pcl::ModelCoefficients> model_coefficients;
		std::vector<pcl::PointIndices> inlier_indices;

		// Estimate Normals
		pcl::IntegralImageNormalEstimation<PointType, pcl::Normal> ne;
		ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
		ne.setMaxDepthChangeFactor (0.02f);
		ne.setNormalSmoothingSize (20.0f);
		ne.setInputCloud (cloud);
		ne.compute (*normal_cloud);

		// Segment planes
		pcl::OrganizedMultiPlaneSegmentation< PointType, pcl::Normal, pcl::Label > mps;
		mps.setMinInliers (10000);
		mps.setAngularThreshold (pcl::deg2rad (2.0)); // 3 degrees
		mps.setDistanceThreshold (0.02); // 1.5 cm

		mps.setInputNormals (normal_cloud);
		mps.setInputCloud (cloud);
		if (use_planar_refinement_)
		{
			mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
		}
		else
		{
			mps.segment (regions);
		}
	}

	void ObjectSegmentation(pcl::PointCloud<PointType>::Ptr &cloud)
	{
		// Segment Objects
		if (regions.size () > 0)
		{
			std::vector<bool> plane_labels;
			plane_labels.resize (label_indices.size (), false);
			for (size_t i = 0; i < label_indices.size (); i++)
			{
				if (label_indices[i].indices.size () > 10000)
				{
					plane_labels[i] = true;
				}
			} 

			euclidean_cluster_comparator_->setInputCloud (cloud);
			euclidean_cluster_comparator_->setLabels (labels);
			euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
			euclidean_cluster_comparator_->setDistanceThreshold (0.01f, false);

			pcl::PointCloud<pcl::Label> euclidean_labels;
			std::vector<pcl::PointIndices> euclidean_label_indices;
			pcl::OrganizedConnectedComponentSegmentation<PointType,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
			euclidean_segmentation.setInputCloud (cloud);
			euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);

			for (size_t i = 0; i < euclidean_label_indices.size (); i++)
			{
				if (euclidean_label_indices[i].indices.size () > 1000)
				{
					pcl::PointCloud<PointType> cluster;
					pcl::copyPointCloud (*cloud,euclidean_label_indices[i].indices,cluster);
					clusters.push_back (cluster);
				}    
			}

			PCL_INFO ("Got %d euclidean clusters!\n", clusters.size ());
		}
		else 
		{
			PCL_INFO ("Can't find euclidean clusters!\n");
		}
	}

	void FindSegmentPlaneScene(pcl::PointCloud<PointType>::Ptr &cloud, pcl::PointCloud<PointType>::Ptr &segment_scene)
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

		PlaneSegmentation(cloud);
		ObjectSegmentation(cloud);	

		for (size_t i = 0; i < clusters.size (); i++)
		{
			*segment_scene = *segment_scene + clusters[i];
		}

#ifdef count_time
		gettimeofday(&tpend,NULL);
		//QueryPerformanceCounter(&nEndTime);
		//time1=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart;
		time1=1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec;
		cout << "Segment time: " << time1 << endl;
		cout << "-------------------------------" << endl;
#endif
	}
	

private:
	std::vector<pcl::PlanarRegion<PointType>, Eigen::aligned_allocator<pcl::PlanarRegion<PointType> > > regions;

	pcl::PointCloud<pcl::Label>::Ptr labels;
	std::vector<pcl::PointIndices> label_indices;
	std::vector<pcl::PointIndices> boundary_indices;
	pcl::PointCloud<PointType>::CloudVectorType clusters;
	pcl::EuclideanClusterComparator<PointType, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_;

};
