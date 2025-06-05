LiDAR Point Cloud Processing Toolkit

<img alt="License: MIT" src="https://img.shields.io/badge/License-MIT-yellow.svg">
A comprehensive toolkit for processing and analyzing LiDAR point cloud data, with specific support for WHU LiDAR format. This toolkit provides utilities for conversion, visualization, segmentation, and analysis of 3D point cloud data.

Features
Format Conversion: Convert between binary LiDAR formats (WHU, KITTI, XYZ) and PCD
Point Cloud Preprocessing: Outlier removal, downsampling, normalization
Feature Extraction: Surface normal estimation
Ground Plane Segmentation: RANSAC-based ground detection
Visualization: Customizable point cloud visualization
Height Map Generation: Create 2D height maps from 3D data

Installation
Prerequisites
Python 3.6+
NumPy
Open3D
Matplotlib
Pathlib

Setup
# Clone the repository
git clone https://github.com/rahulmeganath/lidar.git
cd lidar

```
# Install dependencies
pip install numpy open3d matplotlib
```
Usage
Converting and Visualizing LiDAR Files
```
# Convert bin to PCD and visualize
python whu_lidar.py path/to/lidar/file.bin -v

# Just convert without visualizing
python whu_lidar.py path/to/lidar/file.bin -o output.pcd
```
Interactive Point Cloud Processing
The toolkit includes a Jupyter notebook for interactive point cloud analysis:
```
jupyter notebook plane_segmentation.ipynb
```

Future Work
3D object detection and classification
Multi-frame point cloud registration
Semantic segmentation
Road and drivable area detection
Integration with ROS (Robot Operating System)
Deep learning approaches for feature extraction

License
This project is licensed under the MIT License - see the LICENSE file for details.

Acknowledgments
WHU dataset providers
Open3D development team
