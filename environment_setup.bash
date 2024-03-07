#!/bin/bash

# 定义菜单选项
options=("Install packages." "Check missing packages." "Exit.")

need_setup=0

check_packages(){
  echo "Checking Eigen"
  if [ -d "/usr/include/eigen3" ]; then
    echo "Found!"
    eigen=1
  else
    echo "Not Found!"
    eigen=0
    need_setup=1
  fi

  echo "Checking OpenVINO"
  if [ -d "/usr/include/openvino" ]; then
    echo "Found!"
    openvino=1
  else
    echo "Not Found!"
    openvino=0
    need_setup=1
  fi

  echo "Checking OpenCV"
  if [ -d "/usr/include/opencv4" ]; then
    echo "Found!"
    opencv=1
  else
    echo "Not Found!"
    opencv=0
    need_setup=1
  fi

  echo "Checking Ros2 Humble"
  if [ -d "/opt/ros/humble" ]; then
    echo "Found!"
    ros=1
  else
    echo "Not Found!"
    ros=0
    need_setup=1
  fi

  echo "Checking Ceres"
  if [ -d "/usr/local/include/ceres" ]; then
    echo "Found!"
    ceres=1
  else
    echo "Not Found!"
    ceres=0
    need_setup=1
  fi

  echo "Checking cv_bridge"
  if [ -d "/opt/ros/humble/share/cv_bridge" ]; then
    echo "Found!"
    cv_bridge=1
  else
    echo "Not Found!"
    cv_bridge=0
    need_setup=1
  fi

  echo "Checking MVS"
  if [ -d "/opt/MVS" ]; then
    echo "Found!"
    mvs=1
  else
    echo "Not Found!"
    mvs=0
    need_setup=1
  fi

  if [ $need_setup -eq 0 ]; then
    echo "All packages have been installed!"
  fi

}

install_package(){
  if [ $eigen -eq 0 ]; then
    clear
    echo "Install Eigen..."
    sudo apt install libeigen3-dev -y
  fi

  if [ $openvino -eq 0 ]; then
    clear
    echo "Install openvino..."
    wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
    sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
    echo "deb https://apt.repos.intel.com/openvino/2023 ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2023.list
    sudo apt update
    sudo apt install openvino -y
  fi

  if [ $opencv -eq 0 ]; then
    clear
    echo "Install openCV..."
    sudo apt-get install build-essential python3-numpy python3-pandas cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff5-dev libdc1394-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev liblapacke-dev libxvidcore-dev libx264-dev libatlas-base-dev gfortran ffmpeg -y
    #git clone git@github.com:opencv/opencv.git /tmp/rm/opencv 
    sudo mv /tmp/rm/opencv /usr/local/src
    #git clone git@github.com:opencv/opencv_contrib.git /tmp/rm/opencv_contrib 
    sudo mv /tmp/rm/opencv_contrib /usr/local/src/opencv
    cd /usr/local/src
    cd opencv
    mkdir build
    cd build
    cmake -D CMAKE_INSTALL_PREFIX=/usr/local -D CMAKE_BUILD_TYPE=build -D OPENCV_EXTRA_MODULES_PATH=./opencv_contrib/modules ..
    make -j12
    sudo make install
    sudo ln -s /usr/local/include/opencv4/opencv2/ /usr/local/include/
  fi

  if [ $ros -eq 0 ]; then
    clear
    echo "Install Ros2..."
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe
    sudo apt update
    sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt upgrade
    sudo apt install ros-humble-desktop -y
    sudo apt install ros-dev-tools -y
  fi

  if [ $ceres -eq 0 ]; then
    clear
    echo "Install Ceres..."
    sudo -E git clone https://ceres-solver.googlesource.com/ceres-solver /usr/local/src/ceres-solver
    sudo apt-get install libgoogle-glog-dev libgflags-dev -y
    # Use ATLAS for BLAS & LAPACK
    sudo apt-get install libatlas-base-dev -y
    # SuiteSparse (optional)
    sudo apt-get install libsuitesparse-dev -y
    cd /usr/local/src/ceres-solver
    mkdir ceres-bin
    cd ceres-bin
    cmake ..
    make -j12
    make test
    # Optionally install Ceres, it can also be exported using CMake which
    # allows Ceres to be used without requiring installation, see the documentation
    # for the EXPORT_BUILD_DIR option for more information.
    sudo make install
  fi

  if [ $cv_bridge -eq 0 ]; then
    clear
    echo "Install cv_bridge"
    git clone https://github.com/ros-perception/vision_opencv.git -b humble /tmp/rm/vision_opencv 
    sudo mv /tmp/rm/vision_opencv /usr/loc al/src
    cd /usr/local/src/vision_opencv/cv_bridge
    mkdir build && cd build
    cmake -DCMAKE_INSTALL_PREFIX=/opt/ros/humble ..
    sudo make install
  fi

  if [ $mvs -eq 0 ]; then
    clear
    echo "Install MVS"
    cd ~
    mkdir mvs
    cd mvs
    wget https://www.hikrobotics.com/cn2/source/support/software/MVS_STD_GML_V2.1.2_231225.zip
    unzip MVS_STD_GML_V2.1.2_231225.zip
    architecture=$(uname -m)

    # 判断架构
    if [ "$architecture" == "x86_64" ]; then
      sudo dpkg -i MVS-2.1.2_x86_64_*.deb
    elif [ "$architecture" == "i386" ]; then
      sudo dpkg -i MVS-2.1.2_i386_*.deb
    fi
  fi

}

# 显示菜单
select choice in "${options[@]}"; do
  case $choice in
    "Install packages.")
      check_packages
      install_package
      ;;
    "Check missing packages.")
      check_packages
      ;;
    "Exit.")
      echo "Goodbye!"
      break
      ;;
    *)
      echo "Invalid input!"
      ;;
  esac
done
