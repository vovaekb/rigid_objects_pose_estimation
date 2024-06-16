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
};


