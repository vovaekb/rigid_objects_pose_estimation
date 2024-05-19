#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/visualization/pcl_visualizer.h>


namespace pcl_practicing {

    class AlignmentPrerejective {
    public:
        using PointT = pcl::PointXYZ;
        using PointNormalT = pcl::PointNormal;
        using PointCloudT = pcl::PointCloud<PointT>;
        using PointNormalCloudT = pcl::PointCloud<PointNormalT>;
        using FeatureT = pcl::FPFHSignature33;
        using FeatureEstimationT = pcl::FPFHEstimationOMP<PointT, PointNormalT, FeatureT>;
        using FeatureCloudT = pcl::PointCloud<FeatureT>;
        using ColorHandlerT = pcl::visualization::PointCloudColorHandlerCustom<PointNormalT>;

        struct RansacParameters {
            int maximum_iterations = 50000;
            int samples_number = 3;
            int correspondence_randomness = 5;
            float similarity_threshold = 0.95f;
            float inlier_fraction = 0.25f;
        };

        AlignmentPrerejective();

        void align();

    private:
        float voxel_grid_leaf_size = 0.005f;
        float normal_estimation_radius = 0.005f;
        float feature_estimation_radius = 0.025f;
        PointCloudT::Ptr object_cloud;
        PointCloudT::Ptr scene_cloud;
        PointCloudT::Ptr object_aligned;
        PointCloudT::Ptr scene_before_downsampling;
        FeatureCloudT::Ptr object_features;
        FeatureCloudT::Ptr scene_features;
        
        void loadPointClouds();
        void downsample();
        void estimateNormals();
        void estimateFeatures();
        void performAlignment();
        void getFinalPose();
    };

} // namespace pcl_practicing