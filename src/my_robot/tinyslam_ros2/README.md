# tinyslam_ros2

ROS 2 Humble wrapper for **TinySLAM** (CoreSLAM).  
Thuật toán gốc: <https://github.com/OpenSLAM-org/openslam_tinyslam>

---

## Cấu trúc package

```
tinyslam_ros2/
├── include/tinyslam_ros2/
│   └── tinyslam_node.hpp      # ROS2 node header
├── src/
│   └── tinyslam_node.cpp      # ROS2 wrapper (chỉ phần tích hợp)
├── tinyslam_lib/              # ← Copy source gốc vào đây (xem bên dưới)
│   ├── CoreSLAM.h             # đã có sẵn
│   ├── CoreSLAM.c             # clone từ repo gốc
│   ├── CoreSLAM_ext.c
│   ├── CoreSLAM_random.c
│   ├── CoreSLAM_state.c
│   └── CoreSLAM_loop_closing.c
├── launch/
│   └── tinyslam.launch.py
├── config/
│   ├── tinyslam_params.yaml
│   └── tinyslam.rviz
├── CMakeLists.txt
└── package.xml
```

---

## Hướng dẫn cài đặt

### 1. Clone source TinySLAM gốc vào `tinyslam_lib/`

```bash
cd ~/ros2_ws/src/tinyslam_ros2/tinyslam_lib

# Download các file .c từ repo gốc
for f in CoreSLAM.c CoreSLAM_ext.c CoreSLAM_random.c CoreSLAM_state.c CoreSLAM_loop_closing.c; do
  curl -O "https://raw.githubusercontent.com/OpenSLAM-org/openslam_tinyslam/master/$f"
done
```

### 2. Copy package vào workspace

```bash
cp -r tinyslam_ros2/ ~/ros2_ws/src/
```

### 3. Build

```bash
cd ~/ros2_ws
colcon build --packages-select tinyslam_ros2
source install/setup.bash
```

### 4. Chạy

```bash
# Terminal 1: SLAM
ros2 launch tinyslam_ros2 tinyslam.launch.py

# Với simulation time (Gazebo):
ros2 launch tinyslam_ros2 tinyslam.launch.py use_sim_time:=true
```

---

## TF Tree

```
map
 └── base_link          ← published bởi tinyslam_node
      └── laser         ← published bởi robot_state_publisher (URDF)
```

Node **chỉ publish** `map → base_link`.  
Transform `base_link → laser` lấy từ URDF qua `robot_state_publisher` trong workspace của bạn.

---

## Topics

| Topic  | Type                        | Direction |
|--------|-----------------------------|-----------|
| `/scan`| `sensor_msgs/LaserScan`     | Subscribe |
| `/map` | `nav_msgs/OccupancyGrid`    | Publish   |

---

## Tham số chính (`config/tinyslam_params.yaml`)

| Param                       | Default | Mô tả                                      |
|-----------------------------|---------|--------------------------------------------|
| `sigma_xy`                  | 100.0   | Độ không chắc chắn tịnh tiến (mm)         |
| `sigma_theta`               | 0.1745  | Độ không chắc chắn quay (rad, ~10°)       |
| `hole_width`                | 600     | Độ rộng beam trong map update (mm)         |
| `max_range`                 | 8.0     | Giới hạn tầm xa LiDAR (m)                 |
| `update_map_every_n_scans`  | 1       | Publish map mỗi N scan                    |

---

## Ghi chú thiết kế

- **Không sửa thuật toán**: Toàn bộ logic SLAM nằm trong `tinyslam_lib/` (code C gốc).  
- **Wrapper tối giản**: `tinyslam_node.cpp` chỉ làm 3 việc: convert message, gọi API, publish output.  
- **Realtime ~10 Hz**: Mỗi scan callback chạy `ts_iterative_map_building()` — overhead ROS rất nhỏ.
