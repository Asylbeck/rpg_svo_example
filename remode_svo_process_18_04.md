## First install NVIDIA driver:
### add nvidia ppa:
```sudo add-apt-repository ppa:graphics-drivers/ppa```

### install nvidia driver (I had 440 installed)
```sudo apt-get install nvidia-driver-440```
## Install CUDA
### make sure u delete existing cuda version like following
```sudo apt-get purge cuda```
```sudo apt-get purge libcudnn6```
```sudo apt-get purge libcudnn6-dev```

## Download the appropriate CUDA version:
## CUDA 11 Download Link
```https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1804&target_type=deblocal```

### install following the instructions in the link above or steps below:
```wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin```
```sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600```
```wget http://developer.download.nvidia.com/compute/cuda/11.0.1/local_installers/cuda-repo-ubuntu1804-11-0-local_11.0.1-450.36.06-1_amd64.deb```
```sudo dpkg -i cuda-repo-ubuntu1804-11-0-local_11.0.1-450.36.06-1_amd64.deb```
```sudo apt-key add /var/cuda-repo-ubuntu1804-11-0-local/7fa2af80.pub```
```sudo apt-get update```
```sudo apt-get -y install cuda```
### Restart your PC and run ‘nvidia-smi’ in the terminal
### you should see driver version top left and cuda version top right
##

## Install REMODE dependencies REMODE_BUILD - except google test
## Install boost and eigen
```sudo apt-get install libopencv-dev libeigen3-dev libboost-filesystem-dev```
##install sophus
```cd workspace```
```git clone https://github.com/strasdat/Sophus.git```
```cd Sophus```
```git checkout a621ff```
```mkdir build```
```cd build```
```cmake ..```
```make```
### if errors occur:
### find - so2.cpp replace line 32-33 with 
```unit_complex_ = std::complex<double>(1,0);```
### from sophus folder: sudo ln -s se3.hpp se3.h    /usr/local/include/sophus 
### if no more errors occur: ‘sudo make install’
### install catkin tools (preferred)
```sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'```
```wget http://packages.ros.org/ros.key -O - | sudo apt-key add -```
```sudo apt-get update```
```sudo apt-get install python-catkin-tools```
### Copy SVO 2.0 binaries and unpack them
### in your home folder: make svo_install_overlay_ws and src folder inside it cd into svo_install_overlay_ws/src and clone repos into them
```git clone https://github.com/Asylbeck/rpg_svo_example.git```
### login: gduw
### pw: gdlsUW2020
```git checkout   ubuntu_18```
### copy these libs and unzip
### run the script inside - it will copy some libs inside your ros libs folder so it will request sudo password
```https://drive.google.com/file/d/1Z3WQk3e8iNBW26tAgKbqfCEjDeCQlBPH/view?usp=sharing```
####download regular binaries from: [zurich_svo_2_binaries](http://rpg.ifi.uzh.ch/svo2/svo_binaries_1604_kinetic.zip)
### unzip and copy svo_install_ws to your home folder:
```cp -r <extracted folder>/svo_install_ws/  ~/```
```cd ~```
```cd svo_install_ws```
```./fix_path.sh```
~/svo_install_ws$ chmod 775 -R *

### (it may complain a bit but that’s ok)
### inside svo_install_ws/install/ open _setup_util.py and on line 265 change ‘kinetic’ to ‘melodic’
```source svo_install_ws/install/setup.bash```
### install catkin simple into ur catkin workspace
```cd ~/svo_install_overlay_ws/src/```
```git clone https://github.com/catkin/catkin_simple.git```
### go to the svo catkin workspace (the one you git cloned) and build

### change directory to svo overlay workspace and build
```cd ~/svo_install_overlay_ws/```
### ```catkin build``` or ```catkin_make``` if the catkin tools are not installed
### source 
```source ~/svo_install_overlay_ws/devel/setup.bash```
### launch svo
```roslaunch svo_ros mynteye_mono_imu.launch```

### create a separate catkin worksapce for remode:
```cd ~/```
```mkdir remode_ws && cd remode_ws && mkdir src && catkin build```
## Download an build remode
### in you home folder make a folder remode_ws and a src folder inside of it, cd into src folder and clone remode:
```git clone https://github.com/Asylbeck/rpg_open_remode.git```
### login: glow
### pw: gdlsUW2020
```git checkout ubuntu_1804_mynt```
### change directory to one level up and build with ‘catkin_make’ or ‘catkin build’
### if ```catkin simple``` package is missing then git clone this into your catkin workspace and rebuild
```git clone https://github.com/catkin/catkin_simple.git```
### Launch REMODE and check for errors with CUDA 
```roslaunch rpg_open_remode mynteye.launch```


###Camera - once you follow all of the directions in the https://mynt-eye-d-sdk.readthedocs.io/en/latest/sdk/install_ubuntu_src.html#install-sdk-dependencies, from inside the folder source and launch it
```source wrappers/ros/devel/setup.bash ```
```sudo usermod -a -G video $USER```
