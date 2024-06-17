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

    TEST(AlignmentPrerejectiveTest, LoadObjectFeatures) {
        std::string scene_cloud_file = "scene.pcd";
        std::string object_cloud_file = "object.pcd";
        AlignmentPrerejective alignment;
        alignment.loadScenePointCloud(scene_cloud_file);
        alignment.loadObjectPointCloud(object_cloud_file);
        alignment.align();
        auto object_features = alignment.getObjectFeatures();
        EXPECT_TRUE(static_cast<int>(object_features->size()) > 0);
    }

    TEST(AlignmentPrerejectiveTest, LoadSceneFeatures) {
        std::string scene_cloud_file = "scene.pcd";
        std::string object_cloud_file = "object.pcd";
        AlignmentPrerejective alignment;
        alignment.loadScenePointCloud(scene_cloud_file);
        alignment.loadObjectPointCloud(object_cloud_file);
        alignment.align();
        auto scene_features = alignment.getSceneFeatures();
        EXPECT_TRUE(static_cast<int>(scene_features->points.size()) > 0);
    }
};


