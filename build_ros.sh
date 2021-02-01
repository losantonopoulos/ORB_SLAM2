echo "Building ROS nodes"

cd /home/ubuntu/Packages/ORB_SLAM2/Examples/ROS/ORB_SLAM2
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j2
