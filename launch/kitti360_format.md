the root is KITTI-360


dataset_root in launch file should be "/other_path/KITTI360/"

sequence in launch file should be "2013_05_28_drive_0003_sync/"

note : the "/" in the tail can not be ignored!

    ├── KITTI-360
    │    ├── calibration
    │    │   ├── calib_cam_to_pose.txt
    │    │   ├── calib_cam_to_velo.txt
    │    │   ├── calib_sick_to_velo.txt
    │    │   ├── image_02.yaml
    │    │   ├── image_03.yaml
    │    │   └── perspective.txt
    │    ├── data_2d_raw
    │    │   ├── 2013_05_28_drive_0003_sync
    │    │   │   ├── image_00
    │    │   │   ├── image_01
    │    │   │   ├── image_02
    │    │   │   └── image_03
    │    │   ├── ...
    │    ├── data_2d_semantics
    │    │   └── train
    │    │       ├── 2013_05_28_drive_0000_sync
    │    │       ├── ...
    │    │       ├── 2013_05_28_drive_train_frames.txt
    │    │       └── 2013_05_28_drive_val_frames.txt
    │    ├── data_2d_test
    │    │   ├── 2013_05_28_drive_0008_sync
    │    │   │   ├── image_00
    │    │   │   └── image_01
    │    │   └── ...
    │    ├── data_3d_bboxes
    │    │   ├── train
    │    │   │   ├── 2013_05_28_drive_0000_sync.xml
    │    │   │   ├── ...
    │    ├── data_3d_raw
    │    │   └── 2013_05_28_drive_0003_sync
    │    │       ├── sick_points
    │    │       └── velodyne_points
    │    |	└── ...
    │    ├── data_3d_semantics
    │    │   ├── test
    │    │   │   ├── 2013_05_28_drive_0008_sync
    │    │   │   └── ...
    │    │   └── train
    │    │       ├── 2013_05_28_drive_0000_sync
    │    │       ├── ...
    │    │       ├── 2013_05_28_drive_train.txt
    │    │       └── 2013_05_28_drive_val.txt
    │    └── data_poses
    │        ├── 2013_05_28_drive_0000_sync
    │        │   ├── cam0_to_world.txt
    │        │   ├── oxts
    │        │   └── poses.txt
    │        ├── ...
