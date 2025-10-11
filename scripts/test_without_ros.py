import numpy as np
import open3d as o3d
import copy
import argparse
import sys

# RUN COMMAND:
# python test_without_ros.py local --source /path/to/your/source.pcd --target /path/to/your/target.pcd

DISPLAY = True      # if you run in ssh, set as False

def visualize_registration(src, tar, transformation, window_title):
    if not DISPLAY:
        print("DISPLAY is False, return.")
        return 
    
    src_transformed = copy.deepcopy(src)
    src_transformed.transform(transformation)
    
    src_transformed.paint_uniform_color([1, 0.706, 0])
    tar.paint_uniform_color([0, 0.651, 0.929])
    
    o3d.visualization.draw_geometries([src_transformed, tar], window_name=window_title)

def main():
    parser = argparse.ArgumentParser(description="Point Cloud Registration using ICP.")
    parser.add_argument(
        'mode', 
        type=str, 
        choices=['demo', 'local'],
        help="Working mode: 'demo' Use official demo data | 'local' load from local files。"
    )
    parser.add_argument(
        '--source', '-s', 
        type=str, 
        help="Path of soure point cloud (Olny needed in 'local' mode)"
    )
    parser.add_argument(
        '--target', '-t', 
        type=str, 
        help="Path of target point cloud (Only needed 'local' mode)"
    )
    args = parser.parse_args()

    if args.mode == 'demo':
        print("--- Working mode is [Demo] ---")
        pcd = o3d.data.DemoICPPointClouds()
        src = o3d.io.read_point_cloud(pcd.paths[0])
        tar = o3d.io.read_point_cloud(pcd.paths[1])
        
        trans_init = np.array([
            [0.862, 0.011, -0.507, 0.5], 
            [-0.139, 0.967, -0.215, 0.7],
            [0.487, 0.255, 0.835, -1.4], 
            [0.0, 0.0, 0.0, 1.0]
        ])

    else: # mode == 'local'
        print("--- Working mode is [Local] ---")
        if not args.source or not args.target:
            print("Error: Under 'local' mode, must appoint `--source` and `--target` params")
            sys.exit(1)

        print(f"Load source point cloud: {args.source}")
        print(f"Load target point cloud: {args.target}")
        
        src = o3d.io.read_point_cloud(args.source)
        tar = o3d.io.read_point_cloud(args.target)

        if not src.has_points() or not tar.has_points():
            print("Error: one of file is empty")
            sys.exit(1)

        trans_init = np.identity(4)

    threshold = 0.02
    pipreg = o3d.pipelines.registration

    visualize_registration(src, tar, trans_init, "Initial Alignment")

    print("ICP...")
    reg_p2p = pipreg.registration_icp(
        src, tar, threshold, trans_init,
        pipreg.TransformationEstimationPointToPoint()
    )

    print("\nICP Complated。")
    print(reg_p2p.transformation)

    visualize_registration(src, tar, reg_p2p.transformation, "ICP Final Alignment")

if __name__ == "__main__":
    main()