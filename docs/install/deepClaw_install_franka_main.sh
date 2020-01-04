# !/bin/sh
# download and install 4.12 kernel
./install_rt_kernel.sh
# install realtime kernel
# When asked for the Preemption Model, choose the Fully Preemptible Kernel(5)
# https://frankaemika.github.io/docs
./franka_rt_lib_install_1.sh
# choose 4.12 gernel kernel when reboot
./franka_rt_lib_install_2.sh
# install linfranka
./franka_rt_lib_install_3.sh

# install realsense SDK
./realsenseSDK

# install pip package
./pip_package_install.sh

# install ros
./ROS_install.sh

# download deepclaw
# https://github.com/bionicdl-sustech/DeepClawBenchmark
cd ~/Documents/
git clone https://github.com/bionicdl-sustech/DeepClawBenchmark.git
cd DeepClawBenchmark/
#choose the branch your want to use
#git checkout python2.7
