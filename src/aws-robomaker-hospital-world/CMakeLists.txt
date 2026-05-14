cmake_minimum_required(VERSION 3.8)
project(aws_robomaker_hospital_world)

# 1. Khai báo hệ thống build của ROS 2
find_package(ament_cmake REQUIRED)

# (Tùy chọn) Không bắt buộc phải find_package(gazebo_ros) trong CMakeLists 
# nếu package này chỉ chứa file tài nguyên (models, worlds, launch) và không compile code C++.
# Nhưng nếu bạn muốn giữ lại cho chắc chắn thì có thể bỏ comment dòng dưới:
# find_package(gazebo_ros REQUIRED)

# 2. Thay thế exec_program bằng execute_process (chuẩn CMake mới)
# Lệnh cài đặt thư viện Python từ requirements.txt
execute_process(
  COMMAND python3 -m pip install -r ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Lệnh tải các mô hình 3D từ Gazebo Fuel
execute_process(
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/fuel_utility.py download -m XRayMachine -m IVStand -m BloodPressureMonitor -m BPCart -m BMWCart -m CGMClassic -m StorageRack -m Chair -m InstrumentCart1 -m Scrubs -m PatientWheelChair -m WhiteChipChair -m TrolleyBed -m SurgicalTrolley -m PotatoChipChair -m VisitorKidSit -m FemaleVisitorSit -m AdjTable -m MopCart3 -m MaleVisitorSit -m Drawer -m OfficeChairBlack -m ElderLadyPatient -m ElderMalePatient -m InstrumentCart2 -m MetalCabinet -m BedTable -m BedsideTable -m AnesthesiaMachine -m TrolleyBedPatient -m Shower -m SurgicalTrolleyMed -m StorageRackCovered -m KitchenSink -m Toilet -m VendingMachine -m ParkingTrolleyMin -m PatientFSit -m MaleVisitorOnPhone -m FemaleVisitor -m MalePatientBed -m StorageRackCoverOpen -m ParkingTrolleyMax -d ${CMAKE_CURRENT_SOURCE_DIR}/fuel_models --verbose
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# 3. Copy các thư mục cần thiết vào trong Install/Share của ROS 2
install(
  DIRECTORY launch models fuel_models worlds
  DESTINATION share/${PROJECT_NAME}
)

# 4. Lệnh bắt buộc để ROS 2 (colcon) nhận diện cấu trúc package
ament_package()