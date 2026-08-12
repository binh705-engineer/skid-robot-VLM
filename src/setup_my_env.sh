#!/bin/bash
set -e   # Dung ngay khi co lenh loi, tranh bao "thanh cong" gia
 
echo "=========================================="
echo " Dang nap lai cac thu vien phu thuoc du an"
echo "=========================================="
 
# --- B1: Update apt (bo qua warning GPG cua repo khong lien quan) ---
sudo apt-get update || true
 
# --- B2: Cai cac package he thong / ROS qua apt ---
# (Bao gom ca cac goi cho EKF va pointpillarsnet)
sudo apt-get install -y \
    nano \
    ros-humble-tf-transformations \
    ros-humble-robot-localization \
    ros-humble-magic-enum \
    ros-humble-nmea-msgs \
    ros-humble-rtcm-msgs \
    ros-humble-foxglove-msgs \
    libgeographic-dev \
    ros-humble-velodyne-pointcloud \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-diagnostic-updater \
    ros-humble-message-filters \
    ros-humble-pcl-conversions 

# --- B3: Cai thu vien Python qua pip (khong phai package apt) ---
echo "Dang cai pymodbus (ban <3.0 de tuong thich code dang dung API cu)..."
pip install "pymodbus<3.0" --break-system-packages
pip install "numpy<1.24" --break-system-packages
pip install filterpy --break-system-packages
pip install requests --break-system-packages

# --- B4: Cap quyen chmod cho cong USB, ACM ---
for dev in /dev/ttyACM0 /dev/ttyUSB0; do
    if [ -e "$dev" ]; then
        sudo chmod 666 "$dev"
        echo "Da cap quyen cho $dev"
    else
        echo "Khong thay $dev (chua cam thiet bi hoac chua duoc container nhan)"
    fi
done

# --- B4b: Nap duong dan thu vien tu build tay (OSQP, OsqpEigen...) ---
export CMAKE_PREFIX_PATH=/workspaces/isaac_ros-dev/.local:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/workspaces/isaac_ros-dev/.local/lib:$LD_LIBRARY_PATH
echo "Da nap CMAKE_PREFIX_PATH / LD_LIBRARY_PATH cho OSQP/OsqpEigen"

# --- B5: Nap moi truong ROS 2 ---
echo "Dang nap file setup.bash va local_setup.bash..."
source install/setup.bash
source install/local_setup.bash
echo "Da nap moi truong thanh cong!"
