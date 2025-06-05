import numpy as np
import open3d as o3d
from pathlib import Path
import sys
import argparse

def convert_whu_bin_to_pcd(bin_file_path, pcd_file_path=None):
    """
    Convert a WHU LiDAR .bin file to .pcd format
    
    Args:
        bin_file_path: path to the .bin file
        pcd_file_path: path where to save the .pcd file. If None, replaces .bin extension with .pcd
    
    Returns:
        pcd_file_path: path where the .pcd file was saved
        pcd: Open3D point cloud object
    """
    if pcd_file_path is None:
        pcd_file_path = str(Path(bin_file_path).with_suffix('.pcd'))
    
    # Define the WHU LiDAR binary format structure
    whu_dtype = np.dtype([
        ('x', np.float32), 
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
        ('ring', np.uint16),
        ('time', np.float32)
    ])
    
    try:
        # Read binary data with WHU format
        scan = np.fromfile(bin_file_path, dtype=whu_dtype)
        print(f"Successfully loaded file as WHU format with {len(scan)} points")
    except Exception as e:
        print(f"Error loading as WHU format: {e}")
        print("Trying alternative formats...")
        
        # Try standard KITTI format
        try:
            data = np.fromfile(bin_file_path, dtype=np.float32)
            if len(data) % 4 == 0:
                scan = data.reshape(-1, 4)  # x, y, z, intensity
                points_xyz = scan[:, :3]
                intensity = scan[:, 3]
                print(f"File loaded as KITTI format with {len(points_xyz)} points")
            elif len(data) % 3 == 0:
                points_xyz = data.reshape(-1, 3)  # x, y, z
                intensity = None
                print(f"File loaded as XYZ format with {len(points_xyz)} points")
            else:
                raise ValueError(f"Data size {len(data)} not compatible with known formats")
        except Exception as e2:
            print(f"Failed to load file with alternative formats: {e2}")
            return None, None
    
    # Extract points and other attributes if using WHU format
    if 'scan' in locals() and isinstance(scan, np.ndarray) and scan.dtype == whu_dtype:
        points_xyz = np.vstack([scan['x'], scan['y'], scan['z']]).T
        intensity = scan['intensity']
    
    # Print point cloud stats
    print(f"Point cloud stats: {len(points_xyz)} points")
    if len(points_xyz) > 0:
        print(f"X range: {np.min(points_xyz[:, 0]):.6f} to {np.max(points_xyz[:, 0]):.6f}")
        print(f"Y range: {np.min(points_xyz[:, 1]):.6f} to {np.max(points_xyz[:, 1]):.6f}")
        print(f"Z range: {np.min(points_xyz[:, 2]):.6f} to {np.max(points_xyz[:, 2]):.6f}")
    
    # Check for invalid values
    valid_mask = ~(np.isnan(points_xyz).any(axis=1) | np.isinf(points_xyz).any(axis=1))
    if np.sum(valid_mask) < len(points_xyz):
        print(f"Removing {len(points_xyz) - np.sum(valid_mask)} invalid points")
        points_xyz = points_xyz[valid_mask]
        if intensity is not None:
            intensity = intensity[valid_mask]
    
    # Check if values are extremely large and normalize if needed
    if np.abs(points_xyz).max() > 1e6:
        print("Values are extremely large - normalizing point cloud...")
        
        # Normalize to [-1, 1] range
        points_centered = points_xyz - np.mean(points_xyz, axis=0)
        scale = np.max(np.abs(points_centered))
        if scale > 0:
            points_normalized = points_centered / scale
            points_xyz = points_normalized
            print("Normalized point cloud:")
            print(f"X range: {np.min(points_xyz[:, 0]):.6f} to {np.max(points_xyz[:, 0]):.6f}")
            print(f"Y range: {np.min(points_xyz[:, 1]):.6f} to {np.max(points_xyz[:, 1]):.6f}")
            print(f"Z range: {np.min(points_xyz[:, 2]):.6f} to {np.max(points_xyz[:, 2]):.6f}")
    
    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz)
    
    # Add colors from intensity if available
    if intensity is not None:
        if np.max(intensity) > 0:
            intensity = intensity / np.max(intensity)
        
        colors = np.zeros((len(points_xyz), 3))
        colors[:, 0] = intensity  # Red channel
        colors[:, 1] = intensity  # Green channel
        colors[:, 2] = intensity  # Blue channel
        pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # Save as PCD
    o3d.io.write_point_cloud(pcd_file_path, pcd)
    print(f"Saved point cloud to {pcd_file_path}")
    
    return pcd_file_path, pcd

def visualize_pcd(pcd):
    """Visualize point cloud with coordinate frame"""
    if isinstance(pcd, str):
        pcd = o3d.io.read_point_cloud(pcd)
        
    # Create coordinate frame for reference
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    
    # Visualize
    print("Visualizing point cloud... (press Esc to close)")
    o3d.visualization.draw_geometries([pcd, coordinate_frame])

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert WHU LiDAR bin files to PCD format")
    parser.add_argument("bin_file", help="Path to the .bin file")
    parser.add_argument("-o", "--output", help="Output PCD file path", default=None)
    parser.add_argument("-v", "--visualize", action="store_true", help="Visualize the point cloud after conversion")
    
    args = parser.parse_args()
    
    pcd_path, pcd = convert_whu_bin_to_pcd(args.bin_file, args.output)
    
    if args.visualize and pcd is not None:
        visualize_pcd(pcd)