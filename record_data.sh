#! /bin/bash
ros2 bag record -o "$(date +'%Y%m%d_%H%M%S')" \
/color/camera_info \
/color/image_raw \
/color/metadata \
/depth/camera_info \
/depth/image_rect_raw \
/depth/metadata \
/extrinsics/depth_to_color \
/extrinsics/depth_to_infra1 \
/extrinsics/depth_to_infra2 \
/pf/viz/inferred_pose