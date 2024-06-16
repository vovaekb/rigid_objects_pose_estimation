# rigid_objects_pose_estimation

📍📐 C++ CLI application implementing pipeline for finding the alignment pose of a rigid object in a scene with clutter and occlusions following [tutorial](https://pcl.readthedocs.io/projects/tutorials/en/master/alignment_prerejective.html) from PCL documentation. 

## Using package

Stack:
- PCL (Point Cloud Library)
- Eigen
- Conan

Install:

```
conan install .
conan build .
```

## Run tests

Move to folder bin and run tests:

```
cd bin && ./library_test
```
