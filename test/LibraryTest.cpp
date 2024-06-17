#include <string>
#include <iostream>
#include <filesystem>

#include <gtest/gtest.h>

#include "AlignmentPrerejective.h"

using namespace std::filesystem;

using namespace pcl_practicing;

namespace pcl_practicing {
    TEST(AlignmentPrerejectiveTest, Initial) {
        std::string scene_cloud_file = "scene.pcd";
        std::string object_cloud_file = "object.pcd";
        AlignmentPrerejective alignment;
        alignment.loadScenePointCloud(scene_cloud_file);
        alignment.loadObjectPointCloud(object_cloud_file);
        alignment.align();

        ASSERT_TRUE(true);
    }

    TEST(AlignmentPrerejectiveTest, LoadObjectCloud) {
        std::string scene_cloud_file = "scene.pcd";
        std::string object_cloud_file = "object.pcd";
        AlignmentPrerejective alignment;
        alignment.loadScenePointCloud(scene_cloud_file);
        alignment.loadObjectPointCloud(object_cloud_file);
        alignment.align();
        auto object_features = alignment.getScenePointCloud();
        EXPECT_TRUE(static_cast<int>(object_features->size()) > 0);
    }

    TEST(AlignmentPrerejectiveTest, LoadSceneCloud) {
        std::string scene_cloud_file = "scene.pcd";
        std::string object_cloud_file = "object.pcd";
        AlignmentPrerejective alignment;
        alignment.loadScenePointCloud(scene_cloud_file);
        alignment.loadObjectPointCloud(object_cloud_file);
        alignment.align();
        auto scene_cloud = alignment.getObjectPointCloud();
        EXPECT_TRUE(static_cast<int>(scene_cloud->points.size()) > 0);
    }

    TEST(AlignmentPrerejectiveTest, EstimateObjectFeatures) {
        std::string scene_cloud_file = "scene.pcd";
        std::string object_cloud_file = "object.pcd";
        AlignmentPrerejective alignment;
        alignment.loadScenePointCloud(scene_cloud_file);
        alignment.loadObjectPointCloud(object_cloud_file);
        alignment.align();
        auto object_cloud = alignment.getObjectFeatures();
        EXPECT_TRUE(static_cast<int>(object_cloud->size()) > 0);
    }

    TEST(AlignmentPrerejectiveTest, EstimateSceneFeatures) {
        std::string scene_cloud_file = "scene.pcd";
        std::string object_cloud_file = "object.pcd";
        AlignmentPrerejective alignment;
        alignment.loadScenePointCloud(scene_cloud_file);
        alignment.loadObjectPointCloud(object_cloud_file);
        alignment.align();
        auto scene_features = alignment.getSceneFeatures();
        EXPECT_TRUE(static_cast<int>(scene_features->points.size()) > 0);
    }

    TEST(AlignmentPrerejectiveTest, AlignmentPose) {
        std::string scene_cloud_file = "scene.pcd";
        std::string object_cloud_file = "object.pcd";
        AlignmentPrerejective alignment;
        alignment.loadScenePointCloud(scene_cloud_file);
        alignment.loadObjectPointCloud(object_cloud_file);
        auto pose = alignment.align();
        EXPECT_EQ(pose.rows(), 4);
        EXPECT_EQ(pose.cols(), 4);
    }
};


