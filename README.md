# rigid_objects_pose_estimation

C++ CLI application implementing pipeline for finding the alignment pose of a rigid object in a scene with clutter and occlusions following [tutorial](https://pcl.readthedocs.io/projects/tutorials/en/master/alignment_prerejective.html) from PCL documentation. 

## Using package
Compiling catkin workspase:

```
cd ~/rigid_objects_pose_estimation
mkdir build && cd build
cmake ..
make
