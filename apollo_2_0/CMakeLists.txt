cmake_minimum_required(VERSION 3.8)
project(apollo_2_0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# exec_program("python3 -m pip install -r ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt")
# exec_program("python3 ${CMAKE_CURRENT_SOURCE_DIR}/fuel_utility.py download -m XRayMachine -m IVStand -m BloodPressureMonitor -m BPCart -m BMWCart -m CGMClassic -m StorageRack -m Chair -m InstrumentCart1 -m Scrubs -m PatientWheelChair -m WhiteChipChair -m TrolleyBed -m SurgicalTrolley -m PotatoChipChair -m VisitorKidSit -m FemaleVisitorSit -m AdjTable -m MopCart3 -m MaleVisitorSit -m Drawer -m OfficeChairBlack -m ElderLadyPatient -m ElderMalePatient -m InstrumentCart2 -m MetalCabinet -m BedTable -m BedsideTable -m AnesthesiaMachine -m TrolleyBedPatient -m Shower -m SurgicalTrolleyMed -m StorageRackCovered -m KitchenSink -m Toilet -m VendingMachine -m ParkingTrolleyMin -m PatientFSit -m MaleVisitorOnPhone -m FemaleVisitor -m MalePatientBed -m StorageRackCoverOpen -m ParkingTrolleyMax -d ${CMAKE_CURRENT_SOURCE_DIR}/fuel_models --verbose")

add_executable(initial_pose_publisher src/initial_pose_publisher.cpp)
ament_target_dependencies(initial_pose_publisher rclcpp geometry_msgs tf2)

install(TARGETS
  initial_pose_publisher
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch src urdf meshes config rviz models fuel_models worlds
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
