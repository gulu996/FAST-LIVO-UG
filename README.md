cd ~
git clone https://github.com/borglab/gtsam.git
cd gtsam

git checkout 4.2

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DGTSAM_BUILD_UNSTABLE=ON \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_CONVENIENCE_LIBRARIES=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_PYTHON=OFF

cmake --build build -j2
sudo cmake --install build
sudo ldconfig

cd ~/catkin_ws/src/livox_ros_driver2
./build.sh -j2
