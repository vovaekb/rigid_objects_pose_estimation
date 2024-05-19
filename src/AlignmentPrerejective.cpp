
#include "AlignmentPrerejective.h"

namespace pcl_practicing {
    AlignmentPrerejective::AlignmentPrerejective() : 
        object_cloud (new PointCloudT),
        scene_cloud (new PointCloudT),
        object_aligned (new PointCloudT),
        scene_before_downsampling (new PointCloudT),
        object_features (new FeatureCloudT),
        scene_features (new FeatureCloudT) {}

    void AlignmentPrerejective::align() {}
    void AlignmentPrerejective::loadPointClouds() {}
    void AlignmentPrerejective::downsample() {}
    void AlignmentPrerejective::estimateNormals() {}
    void AlignmentPrerejective::estimateFeatures() {}
    void AlignmentPrerejective::performAlignment() {}
    void AlignmentPrerejective::getFinalPose() {}
};


