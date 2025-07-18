# LiDAR Point Cloud Processing Toolkit

![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)

A comprehensive toolkit for processing and analyzing LiDAR point cloud data, with specific support for **WHU LiDAR** format. This toolkit provides utilities for conversion, visualization, segmentation, and analysis of 3D point cloud data.

---

## ✨ Features

* **Format Conversion**: Convert between binary LiDAR formats (WHU, KITTI, XYZ) and PCD
* **Point Cloud Preprocessing**: Outlier removal, downsampling, normalization
* **Feature Extraction**: Surface normal estimation
* **Ground Plane Segmentation**: RANSAC-based ground detection
* **Visualization**: Customizable point cloud visualization
* **Height Map Generation**: Create 2D height maps from 3D data

---

## 📦 Installation

### Prerequisites

* Python 3.6+
* [NumPy](https://numpy.org/)
* [Open3D](http://www.open3d.org/)
* [Matplotlib](https://matplotlib.org/)
* [Pathlib](https://docs.python.org/3/library/pathlib.html)

### Setup

```bash
# Clone the repository
git clone https://github.com/rahulmeganath/lidar.git
cd lidar

# Install dependencies
pip install numpy open3d matplotlib
```

---

## 🚀 Usage

### Converting and Visualizing LiDAR Files

```bash
# Convert .bin to .pcd and visualize
python whu_lidar.py path/to/lidar/file.bin -v

# Just convert without visualizing
python whu_lidar.py path/to/lidar/file.bin -o output.pcd
```

### Interactive Point Cloud Processing

Launch the included Jupyter notebook for interactive analysis:

```bash
jupyter notebook plane_segmentation.ipynb
```

---

## 🧭 Plane Segmentation Procedures

The **plane segmentation pipeline** implemented in the notebook includes the following steps:

### 1. **Point Cloud Loading and Analysis**

* Load `.pcd` files using Open3D
* Print statistics: point count and spatial dimensions
* Check for presence of colors and normals
* Visualize the raw point cloud

### 2. **Preprocessing Operations**

* **Statistical Outlier Removal**: Remove noisy points using neighborhood statistics
* **Voxel Grid Downsampling**: Reduce point cloud density for efficiency
* Visualize the cleaned, downsampled cloud

### 3. **Feature Extraction**

* **Normal Estimation**: Compute surface normals using a hybrid neighborhood search
* **Normal Orientation**: Orient normals consistently toward a virtual camera
* Visualize normals overlaid on the point cloud

### 4. **Ground Plane Segmentation**

* **RANSAC Implementation**: Detect the dominant ground plane via RANSAC
* **Plane Model Extraction**: Derive the plane equation (ax + by + cz + d = 0)
* **Normal Analysis**: Compare plane normal with the vertical axis
* **Point Classification**: Label points as ground (green) or non-ground (red)
* **Statistics Calculation**: Report percentage of ground vs. non-ground points

### 5. **Segmentation Visualization**

* Render point cloud with color-coded segmentation
* Generate separate views: ground-only and non-ground-only

### 6. **Height Map Generation**

* Compute signed distances from non-ground points to ground plane
* Apply a configurable height filter
* Convert filtered points into a 2D grid representation
* Assign heights to grid cells and visualize as a color-coded map

---

## 🛠️ Future Work

* 3D object detection and classification
* Multi-frame point cloud registration
* Semantic segmentation
* Road and drivable area detection
* Integration with ROS (Robot Operating System)
* Deep learning approaches for feature extraction

---

## 🙏 Acknowledgments

* WHU dataset providers
* Open3D development team

