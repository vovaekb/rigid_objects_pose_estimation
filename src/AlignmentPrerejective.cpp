
#include "AlignmentPrerejective.h"

namespace pcl_practicing {
    AlignmentPrerejective::AlignmentPrerejective() : 
        object_cloud (new PointCloudT),
        scene_cloud (new PointCloudT),
        scene_before_downsampling (new PointCloudT),
        scene_normals (new PointNormalCloudT),
        object_normals (new PointNormalCloudT),
        object_features (new FeatureCloudT),
        scene_features (new FeatureCloudT) {}

    void AlignmentPrerejective::align() {
        downsample();
        estimateNormals();
        estimateFeatures();

        PointCloudT::Ptr object_aligned (new PointCloudT);
        performAlignment(object_aligned);

    }
    void AlignmentPrerejective::loadScenePointCloud(const std::string& file_path) {
       
        pcl::io::loadPCDFile(file_path, *scene_before_downsampling);
        PCL_INFO("Scene cloud has %d points\n", static_cast<int>(scene_before_downsampling->points.size()));
    }

    void AlignmentPrerejective::loadObjectPointCloud(const std::string& file_path) {
        pcl::io::loadPCDFile(file_path, *object_cloud);
        PCL_INFO("object cloud has %d points\n", static_cast<int>(object_cloud->points.size()));
    }
    void AlignmentPrerejective::downsample() {
        pcl::VoxelGrid<PointT> grid;

        grid.setLeafSize (voxel_grid_leaf_size, voxel_grid_leaf_size, voxel_grid_leaf_size);
        // downsample object
        grid.setInputCloud (object_cloud);
        grid.filter (*object_cloud);

        // downsample scene
        grid.setInputCloud (scene_before_downsampling);
        grid.filter (*scene_cloud);

    }
    void AlignmentPrerejective::estimateNormals() {
        pcl::NormalEstimationOMP<PointT, PointNormalT> nest;
        nest.setRadiusSearch(normal_estimation_radius);
        // estimate normals for scene cloud
        nest.setInputCloud(scene_cloud);
        nest.setSearchSurface(scene_before_downsampling);
        nest.compute(*scene_normals);

        // estimate normals for object cloud
        nest.setInputCloud(object_cloud);
        nest.setSearchSurface(object_cloud);
        nest.compute(*object_normals);

    }
    void AlignmentPrerejective::estimateFeatures() {
        FeatureEstimationT fest;
        fest.setRadiusSearch(feature_estimation_radius);
        fest.setInputCloud(object_cloud);
        fest.setInputNormals(object_normals);
        fest.compute(*object_features);

        fest.setInputCloud(scene_cloud);
        fest.setInputNormals(scene_normals);
        fest.compute(*scene_features);

    }

    void AlignmentPrerejective::performAlignment(const PointCloudT::Ptr& object_aligned) {
        PCL_INFO("Perform alignment");
        pcl::SampleConsensusPrerejective<PointT, PointT, FeatureT> alignment;
        alignment.setInputSource(object_cloud);
        alignment.setSourceFeatures(object_features);
        alignment.setInputTarget(scene_cloud);
        alignment.setTargetFeatures(scene_features);
        alignment.setMaximumIterations(RansacParameters::MAXIMUM_ITERATIONS);
        alignment.setNumberOfSamples(RansacParameters::SAMPLES_NUMBER);
        alignment.setCorrespondenceRandomness(RansacParameters::CORRESPONDENCE_RANDOMNESS);
        alignment.setSimilarityThreshold(RansacParameters::SIMILARITY_THRESHOLD);
        alignment.setMaxCorrespondenceDistance(2.5f * voxel_grid_leaf_size);
        alignment.setInlierFraction(RansacParameters::INLIER_FRACTION);
        alignment.align(*object_aligned);
        PCL_INFO("SampleConsensusPrerejective alignment has converged %d\n", static_cast<int>(alignment.hasConverged()));

        auto result_transformation = alignment.getFinalTransformation();

        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", result_transformation (0,0), result_transformation (0,1), result_transformation (0,2));
        pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", result_transformation (1,0), result_transformation (1,1), result_transformation (1,2));
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", result_transformation (2,0), result_transformation (2,1), result_transformation (2,2));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", result_transformation (0,3), result_transformation (1,3), result_transformation (2,3));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("Inliers: %i/%i\n", alignment.getInliers().size(), object_cloud->size());

    }

    auto AlignmentPrerejective::getSceneFeatures() const -> FeatureCloudT::Ptr {
        return scene_features;
    }

    auto AlignmentPrerejective::getObjectFeatures() const -> FeatureCloudT::Ptr {
        return object_features;
    }
    
};
