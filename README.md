# Pose Optimization Using Lines Combined with G2O Library: an Easy Example

## A small sample for pose optimization using g2o to minimize reprojection errors from known lines' correspondences.

### Usage:

Requirement:

1. OpenCV 3.1.0 or newer version.

2. LAPACK && Suitesparse && Eigen3.

3. (optional) Viz module in opencv (needs compiling with VTK). Disable the display in the macro DISPLAY_3D.

Example:

There is an example given on optimizing a pose calculated from a near pose to give intuitions on the performance of line-based SLAM method utlizing g2o. 
