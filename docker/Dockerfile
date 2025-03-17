FROM nvidia/cuda:12.1.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# Essential packages
RUN apt-get update && apt-get install -y \
    wget curl git vim tmux build-essential cmake pkg-config python3 python3-pip python3-dev python3-numpy \
    libgtk2.0-dev libjpeg-dev libtiff5-dev libpng-dev libavcodec-dev libavformat-dev libswscale-dev \
    libdc1394-dev libprotobuf-dev protobuf-compiler libgoogle-glog-dev libgflags-dev \
    libhdf5-dev doxygen libflann-dev libboost-all-dev libyaml-cpp-dev libtbb-dev libopenexr-dev \
    libglfw3-dev libglew-dev libgtest-dev doxygen graphviz

# Eigen 3.4.0
RUN cd / && git clone https://gitlab.com/libeigen/eigen.git && \
    cd eigen && git checkout 3.4.0 && mkdir build && cd build && \
    cmake .. && make install

# OpenCV 4.5.4 from source (with contrib modules and xfeatures2d)
RUN cd / && git clone --branch 4.5.4 https://github.com/opencv/opencv.git && \
    git clone --branch 4.5.4 https://github.com/opencv/opencv_contrib.git && \
    cd opencv && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
          -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
          -DENABLE_NONFREE=ON .. && \
    make -j$(nproc) && make install && ldconfig

# OpenCV 4.5.4 from source (with CUDA + contrib modules)
RUN cd / && git clone --branch 4.5.4 https://github.com/opencv/opencv.git && \
    git clone --branch 4.5.4 https://github.com/opencv/opencv_contrib.git && \
    cd opencv && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX=/usr/local \
          -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
          -DENABLE_NONFREE=ON \
          -DWITH_CUDA=ON \
          -DCUDA_ARCH_BIN=8.6 \ 
          -DCUDA_ARCH_PTX=8.6 \
          -DWITH_CUDNN=ON \
          .. && \
    make -j$(nproc) && make install && ldconfig


# Python packages
RUN pip3 install --upgrade pip && pip3 install scipy joblib scikit-learn ruamel.yaml trimesh pyyaml \
    opencv-python open3d transformations warp-lang einops kornia pyrender \
    scikit-image meshcat webdataset omegaconf pypng roma seaborn openpyxl wandb imgaug

ENV OPENCV_IO_ENABLE_OPENEXR=1

# Copy source code and build
# COPY . /M3T
# RUN cd /M3T && mkdir build && cd build && \
#     cmake .. && make -j$(nproc)

CMD ["/bin/bash"]
