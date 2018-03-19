cd ~

# Install other python libraries and their dependencies
sudo apt-get purge libreoffice wolfram-engine sonic-pi scratch -y
sudo apt-get autoremove -y
sudo apt-get remove --purge libreoffice* -y
sudo apt-get clean -y
sudo apt-get autoremove -y
sudo apt-get dist-upgrade -y
sudo apt-get update -y
sudo apt-get upgrade -y
apt-get install ntfs-3g -y # driver for reading ntfs formatted data
sudo apt-get install libblas-dev        
sudo apt-get install liblapack-dev      
sudo apt-get install python-dev        
sudo apt-get install libatlas-base-dev  
sudo apt-get install gfortran          
sudo apt-get install python-setuptools -y
sudo apt-get install python3-setuptools -y
# This will install sklearn as well as matplotlib and scipy
sudo apt-get install python3-sklearn -y
sudo apt-get install python3-skimage -y
sudo apt-get install python3-pandas -y
sudo apt-get install python3-h5py -y
sudo apt-get install python3-cairocffi -y
sudo pip3 install opencv-python
wget https://github.com/samjabrahams/tensorflow-on-raspberry-pi/releases/download/v1.1.0/tensorflow-1.1.0-cp34-cp34m-linux_armv7l.whl
sudo pip3 install tensorflow-1.1.0-cp35-cp35m-linux_armv7l.whl
sudo pip3 install keras
sudo pip3 install readchar # readchar is actually for controller.py for controlling rccar

# Install ROS for rasbian stretch
sudo apt-get install dirmngr -y
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get install python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake -y
sudo rosdep init
rosdep update
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws
rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
wstool init -j8 src kinetic-ros_comm-wet.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
sudo mkdir -p /opt/ros/kinetic
sudo chown pi:pi /opt/ros/kinetic
./src/catkin/bin/catkin_make_isolated -j2 --install --install-space /opt/ros/kinetic -DCMAKE_BUILD_TYPE=Release
source /opt/ros/kinetic/setup.bash
export | grep ROS 

cd ~
# Setup things for controlling rccar via PS4 controller in python
sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt-get dist-upgrade -y
sudo rpi-update -y
sudo apt-get install bluetooth bluez blueman -y
sudo apt-get install python-pygame -y
sudo pip install ds4drv
sudo pip3 install ds4drv


# Install pigpio library
rm pigpio.tar
sudo rm -rf PIGPIO
wget abyz.me.uk/rpi/pigpio/pigpio.tar
tar xf pigpio.tar
cd PIGPIO
make
sudo make install

# Install opencv for C++
cd ~
sudo apt-get install at-spi2-core -y
sudo apt-get install build-essential cmake cmake-curses-gui pkg-config -y
sudo apt-get install libgdk-pixbuf2.0-dev libpango1.0-dev libcairo2-dev -y
sudo apt-get install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev -y
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev -y
sudo apt-get install libxvidcore-dev libx264-dev -y
sudo apt-get install libgtk2.0-dev -y
sudo apt-get install libatlas-base-dev gfortran -y
wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.3.0.zip
unzip opencv.zip
wget -O opencv_contrib.zip https://github.com/Itseez/opencv_contrib/archive/3.3.0.zip
unzip opencv_contrib.zip
cd ~/opencv-3.3.0/
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D INSTALL_PYTHON_EXAMPLES=ON -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-3.3.0/modules -D BUILD_EXAMPLES=ON ..
make -j4
sudo make install
sudo ldconfig



cd ~

