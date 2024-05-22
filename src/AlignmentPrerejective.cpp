
#include "AlignmentPrerejective.h"

namespace pcl_practicing {
    AlignmentPrerejective::AlignmentPrerejective() : 
        object_cloud (new PointCloudT),
        scene_cloud (new PointCloudT),
        object_aligned (new PointCloudT),
        scene_before_downsampling (new PointCloudT),
        scene_normals (new PointNormalCloudT),
        object_normals (new PointNormalCloudT),
        object_features (new FeatureCloudT),
        scene_features (new FeatureCloudT) {}

    void AlignmentPrerejective::align() {
        // loadPointClouds();
        downsample();
        estimateNormals();
        estimateFeatures();
        performAlignment();
        getFinalPose();
    }
    void AlignmentPrerejective::loadScenePointCloud(const std::string& file_path) {
        std::cout << "load scene Point Cloud from file.\n";
        
        pcl::io::loadPCDFile(file_path, *scene_before_downsampling);
        PCL_INFO("Scene cloud has %d points\n", (int)scene_before_downsampling->points.size());
    }

    void AlignmentPrerejective::loadObjectPointCloud(const std::string& file_path) {
        std::cout << "load object Point Cloud from file.\n";

        pcl::io::loadPCDFile(file_path, *object_cloud);
        PCL_INFO("object cloud has %d points\n", (int)object_cloud->points.size());
    }
    void AlignmentPrerejective::downsample() {
        pcl::console::print_highlight ("Downsampling...\n");
        pcl::VoxelGrid<PointT> grid;

        grid.setLeafSize (voxel_grid_leaf_size, voxel_grid_leaf_size, voxel_grid_leaf_size);
        // downsample object
        grid.setInputCloud (object_cloud);
        grid.filter (*object_cloud);

        PCL_INFO("object cloud has %d points after down sampling\n", (int)object_cloud->points.size());

        // downsample scene
        grid.setInputCloud (scene_before_downsampling);
        grid.filter (*scene_cloud);

        PCL_INFO("Scene cloud has %d points after down sampling\n", (int)scene_cloud->points.size());

    }
    void AlignmentPrerejective::estimateNormals() {
        pcl::NormalEstimationOMP<PointT, PointNormalT> nest;
        nest.setRadiusSearch(normal_estimation_radius);
        // estimate normals for scene cloud
        nest.setInputCloud(scene_cloud);
        nest.setSearchSurface(scene_before_downsampling);
        nest.compute(*scene_normals);

        PCL_INFO("Scene normals cloud has %d points\n", (int)scene_normals->points.size());

        // estimate normals for object cloud
        nest.setInputCloud(object_cloud);
        nest.setSearchSurface(object_cloud);
        nest.compute(*object_normals);

        PCL_INFO("Object normals cloud has %d points\n", (int)object_normals->points.size());
    }
    void AlignmentPrerejective::estimateFeatures() {
        FeatureEstimationT fest;
        fest.setRadiusSearch(feature_estimation_radius);
        fest.setInputCloud(object_cloud);
        fest.setInputNormals(object_normals);
        fest.compute(*object_features);

        PCL_INFO("Object features cloud has %d points\n", (int)object_features->points.size());

        fest.setInputCloud(scene_cloud);
        fest.setInputNormals(scene_normals);
        fest.compute(*scene_features);

        PCL_INFO("Scene features cloud has %d points\n", (int)scene_features->points.size());
    }

    void AlignmentPrerejective::performAlignment() {
        PCL_INFO("Perform alignment");
        pcl::SampleConsensusPrerejective<PointT, PointT, FeatureT> align;
        align.setInputSource(object_cloud);
        align.setSourceFeatures(object_features);
        align.setInputTarget(scene_cloud);
        align.setTargetFeatures(scene_features);
        align.setMaximumIterations(RansacParameters::MAXIMUM_ITERATIONS);
        align.setNumberOfSamples(RansacParameters::SAMPLES_NUMBER);
        align.setCorrespondenceRandomness(RansacParameters::CORRESPONDENCE_RANDOMNESS);
        align.setSimilarityThreshold(RansacParameters::SIMILARITY_THRESHOLD);
        align.setMaxCorrespondenceDistance(2.5f * voxel_grid_leaf_size);
        align.setInlierFraction(RansacParameters::INLIER_FRACTION); // 0.25f);
        align.align(*object_aligned);
        PCL_INFO("SampleConsensusPrerejective alignment has converged %d\n", (int)align.hasConverged());
    }
    void AlignmentPrerejective::getFinalPose() {}
};


