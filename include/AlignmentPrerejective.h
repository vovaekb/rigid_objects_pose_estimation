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
// #include <pcl/visualization/pcl_visualizer.h>


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
        // using ColorHandlerT = pcl::visualization::PointCloudColorHandlerCustom<PointNormalT>;

        struct RansacParameters {
            static constexpr int MAXIMUM_ITERATIONS = 50000;
            static constexpr int SAMPLES_NUMBER = 7; // 3;
            static constexpr int CORRESPONDENCE_RANDOMNESS = 10; // 5;
            static constexpr float SIMILARITY_THRESHOLD = 0.5f; // 0.95f;
            static constexpr float INLIER_FRACTION = 0.05f; // 0.25f;
        };

        AlignmentPrerejective();

        void loadScenePointCloud(const std::string& file_path);

        void loadObjectPointCloud(const std::string& file_path);

        void align();

    private:
        float voxel_grid_leaf_size = 0.005f;
        float normal_estimation_radius = 0.005f;
        float feature_estimation_radius = 0.025f;
        RansacParameters m_ransac_parameters;
        PointCloudT::Ptr object_cloud;
        PointCloudT::Ptr scene_cloud;
        PointCloudT::Ptr object_aligned;
        PointCloudT::Ptr scene_before_downsampling;
        PointNormalCloudT::Ptr scene_normals;
        PointNormalCloudT::Ptr object_normals;
        FeatureCloudT::Ptr object_features;
        FeatureCloudT::Ptr scene_features;

        void downsample();
        void estimateNormals();
        void estimateFeatures();
        void performAlignment();
        void getFinalPose();
    };

} // namespace pcl_practicing