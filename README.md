# My Robot ROS 2 - Omni 3 Wheel Robot with Arm


```bash
git clone https://github.com/VTongg159/omni_wheel_ros2.git
```

```bash
cd Omni_3_wheel_ROS
colcon build
source install/setup.bash 
```

### **Chạy tren Gazebo**

```bash
ros2 launch my_robot_description sim.launch.py use_sim_time:=true
```

### **Cartographer2D**

```bash
ros2 launch my_robot_cartographer cartographer.launch.py use_sim_time:=true
```

### **Hector_slam2D**

```bash
ros2 launch hector_slam_ros2 hector_slam.launch.py 
```

```bash
ros2 run rviz2 rviz2
```

thêm map trong rviz, đặt topic thành /map

### **Karto_slam**

```bash
ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=true

```

```bash
rviz2 -d src/my_robot/slam_karto/config/slam_toolbox_default.rviz use_sim_time:=true

```
thêm map trong rviz, đặt topic thành /map

### **Gmapping**

```bash
ros2 ros2 launch slam_gmapping slam_gmapping.launch.py use_sim_time:=true

```

```bash
ros2 run rviz2 rviz2
```
thêm map trong rviz, đặt topic thành /map



### ** Điều Khiển Robot Omni (xe 3 Bánh)**

Điều khiển chuyển động của robot base (3 bánh omni).

```bash
ros2 run my_robot_teleop omni_teleop
```


### ** Dùng ros2 bag để  quét map cho các thuật toán **

```bash
ros2 launch my_robot_description sim.launch.py use_sim_time:=true
```
Ghi dữ liệu các topic cần thiết cho SLAM (Laser scan, Odometry, và TF)

```bash
ros2 bag record /scan /odom /tf /tf_static
```

Chạy teleop và điều khiển robot chạy hết bản đồ

```bash
ros2 run my_robot_teleop omni_teleop
```

Sau khi xong, nhấn **Ctrl+C** ở terminal ghi bag để lưu file.

Phát file bag:
    
```bash
    ros2 bag play <tên_thư_mục_bag> --clock
```
 Tham số `--clock` để đồng bộ thời gian giữa dữ liệu cũ và hệ thống hiện tại.*

Chạy thuật toán SLAM bạn muốn thử nghiệm:

    Ví dụ, để thử **Cartographer**:
    
```bash
    ros2 launch my_robot_cartographer cartographer.launch.py use_sim_time:=true
```
    Hoặc thử **Slam Toolbox (Karto)**:
    
```bash
    ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=true
```

Mở Rviz2 để xem kết quả:
    ```bash
    ros2 run rviz2 rviz2
    ```

### **Điều Khiển Tay Robot (2 Khớp)**

Điều khiển tay máy bằng bàn phím.

```bash
cd src/my_robot_arm_controller/my_robot_arm_controller
python arm_controller.py
```



