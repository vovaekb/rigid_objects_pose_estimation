/**
 * @file AlignmentPrerejective.h
 * @brief Class for pipeline of finding the alignment pose of a rigid object in a scene 
 */

#pragma once

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


namespace pcl_practicing {

    /**
     * @brief Pipeline for finding the alignment pose of a rigid object in a scene
     */
    class AlignmentPrerejective {
    public:
        using PointT = pcl::PointXYZ;
        using PointNormalT = pcl::PointNormal;
        using PointCloudT = pcl::PointCloud<PointT>;
        using PointNormalCloudT = pcl::PointCloud<PointNormalT>;
        using FeatureT = pcl::FPFHSignature33;
        using FeatureEstimationT = pcl::FPFHEstimationOMP<PointT, PointNormalT, FeatureT>;
        using FeatureCloudT = pcl::PointCloud<FeatureT>;

        /**
         * @brief Struct for Ransac parameters
         */
        struct RansacParameters {
            /**!< Maximum number of iterations */
            static constexpr int MAXIMUM_ITERATIONS = 50000;
            /**!< Samples number */
            static constexpr int SAMPLES_NUMBER = 7;
            /**!< Correspondence randomness */
            static constexpr int CORRESPONDENCE_RANDOMNESS = 10;
            /**!< Similarity threshold */
            static constexpr float SIMILARITY_THRESHOLD = 0.5f;
            /**!< Inlier fraction */
            static constexpr float INLIER_FRACTION = 0.05f;
        };

        AlignmentPrerejective();

        /**
         * @brief Load point cloud of scene from disk
         * @param[in] file_path Path to input point cloud
         * */
        void loadScenePointCloud(const std::string& file_path);

        /**
         * @brief Load point cloud of object from disk
         * @param[in] file_path Path to input point cloud
         * */
        void loadObjectPointCloud(const std::string& file_path);

        /**
         * @brief Start pipeline for finding the alignment pose of a rigid object in a scene
         * */
        void align();

    private:
        /// Leaf size for Voxel Grid downsampling
        float voxel_grid_leaf_size = 0.005f;
        /// Radius for normal estimation
        float normal_estimation_radius = 0.005f;
        /// Radius for feature descriptors estimation
        float feature_estimation_radius = 0.025f;
        /// Ransac parameters
        RansacParameters m_ransac_parameters;
        /// Point cloud of object
        PointCloudT::Ptr object_cloud;
        /// Point cloud of scene
        PointCloudT::Ptr scene_cloud;
        /// Point cloud of scene before downsampling
        PointCloudT::Ptr scene_before_downsampling;
        /// Point cloud of scene normals
        PointNormalCloudT::Ptr scene_normals;
        /// Point cloud of object normals
        PointNormalCloudT::Ptr object_normals;
        /// Point cloud of object feature descriptors
        FeatureCloudT::Ptr object_features;
        /// Point cloud of scene feature descriptors
        FeatureCloudT::Ptr scene_features;

        /**
         * @brief Downsample point cloud
         * */
        void downsample();
        /**
         * @brief Estimate normals on point cloud
         * */
        void estimateNormals();
        /**
         * @brief Estimate feature descriptors on point cloud
         * */
        void estimateFeatures();
        /**
         * @brief Perform alignment of object in scene and get final transformation pose of object in scene
         * @param[in] object_aligned Point cloud of object alignment result
         * */
        void performAlignment(const PointCloudT::Ptr& object_aligned);

        /**
         * @brief Visualize result of alignment
         * */
        void visualizeResult();
    };

} // namespace pcl_practicing