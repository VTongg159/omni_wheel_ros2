# My Robot ROS 2 - Omni 3 Wheel Robot with Arm

Dự án ROS 2 mô phỏng và điều khiển robot di chuyển đa hướng (Omni 3 bánh) kết hợp với cánh tay robot 2 bậc tự do.  
Hệ thống hỗ trợ:

- Điều khiển thủ công
- Mô phỏng Gazebo
- Nhiều thuật toán SLAM 2D
- Mapping bằng `ros2 bag`

---

## 📦 Environment

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- RViz2

---

## 🛠 Cài đặt và Build Project

```bash
# Clone repository
git clone https://github.com/VTongg159/omni_wheel_ros2.git

# Di chuyển vào workspace
cd omni_wheel_ros2

# Build project
colcon build

# Source môi trường
source install/setup.bash
```

---

## Quy trình lập bản đồ bằng ROS 2 Bag

### Bước 1: Khởi chạy mô phỏng

```bash
ros2 launch my_robot_description sim.launch.py use_sim_time:=true
```

---

### Bước 2: Ghi dữ liệu cảm biến

```bash
ros2 bag record /scan /odom /tf /tf_static
```

---

### Bước 3: Điều khiển robot

```bash
ros2 run my_robot_teleop omni_teleop
```

Di chuyển robot quanh môi trường để thu thập dữ liệu mapping.

Sau khi hoàn tất, nhấn `Ctrl + C` để dừng ghi bag, tắt trình mô phỏng gazebo

---

## Chạy các thuật toán SLAM

Chọn một trong các thuật toán SLAM dưới đây để tiến hành tạo bản đồ. (Lưu ý: Mở terminal mới, nhớ source môi trường và thêm use_sim_time:=true)

### Cartographer 2D

```bash
ros2 launch my_robot_cartographer cartographer.launch.py use_sim_time:=true
```

---

### Hector SLAM

```bash
ros2 launch hector_slam_ros2 hector_slam.launch.py use_sim_time:=true

```

---

### SLAM Toolbox (Karto)

```bash
ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=true

```


---

### Gmapping

```bash
ros2 launch slam_gmapping slam_gmapping.launch.py use_sim_time:=true

```

---

## ▶ Phát lại ROS 2 Bag

```bash
ros2 bag play <bag_directory>
```

Quan sát quá trình tạo bản đồ trên RViz.

---

##  Lưu bản đồ

```bash
ros2 run nav2_map_server map_saver_cli -f my_map_name
```

Ví dụ:

```bash
ros2 run nav2_map_server map_saver_cli -f gmapping_waffle_map
```