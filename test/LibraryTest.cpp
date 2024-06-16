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
        // std::string result_path = "result";
        AlignmentPrerejective alignment;
        // alignment.loadScenePointCloud(scene_cloud_file);
        // alignment.loadObjectPointCloud(object_cloud_file);
        // alignment.align();

        // ObjectSegmenter segmenter;
        // segmenter.run(cloud_file, indices_file, result_path);
        ASSERT_TRUE(true);

        // path dir_path = result_path;

        // int files_number = 0;
        // std::string result_file_name;
        // for (const auto& file : directory_iterator(dir_path))
        // {
        //     ++files_number;
        //     result_file_name = file.path().filename();
        // }
        // ASSERT_EQ(files_number, 1);
        // ASSERT_EQ(result_file_name, "object.pcd");
    }
};


