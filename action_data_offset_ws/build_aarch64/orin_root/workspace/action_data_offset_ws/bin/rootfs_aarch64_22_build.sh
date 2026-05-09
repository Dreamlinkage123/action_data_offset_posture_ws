#！/bin/bash

# 不能单独执行，被 build.sh 调用
# 需要 SYSROOT environment_dir 两环境变量

count_success=0
count_fail=0

export is_orin=$(grep -q 'tegra-ubuntu' /etc/hostname && echo "true" || echo "false")
export is_pc=$([[ $is_orin == "true" ]] && echo "false" || echo "true")

pip_server=" -i http://mirrors.aliyun.com/pypi/simple/ --trusted-host mirrors.aliyun.com "

function print_and_exit_if_ret()
{
    ret=$1
    msg=$2
    desc=${@:3}
    if [ $ret -ne 0 ]; then
        echo "########################################### "
        echo "# $msg() ${desc} fail !!!!!! "
        echo "########################################### "
        exit 1   # 直接退出
    fi
}

function mount_mem_fs()
{
	sudo mount -o bind /proc ${SYSROOT}/proc
	sudo mount -o bind /sys ${SYSROOT}/sys
	sudo mount -o bind /dev ${SYSROOT}/dev
    sudo chmod 777 ${SYSROOT}/dev/null
}

function umount_mem_fs()
{
	sudo umount ${SYSROOT}/proc > /dev/null  2>&1
	sudo umount ${SYSROOT}/sys  > /dev/null  2>&1
	sudo umount ${SYSROOT}/dev  > /dev/null  2>&1
}

function EXEC_ROOTFS()
{
    cmd=${@:1}
    echo "# exec_rootfs: $cmd"
    if [ $is_pc == "true" ]; then
        sudo LC_ALL=C chroot ${SYSROOT} bash -c "$cmd"
        ret=$?
    else
        sudo bash -c "$cmd"
        ret=$?
    fi

    if [ $ret -eq 0 ]; then
        let count_success=count_success+1
    else
        let count_fail=count_fail+1
        echo -e "\033[31m# count_fail:$count_fail >>> exec_rootfs: $cmd\033[0m"
    fi
    return $ret
}

function modify_rootfs_2204_aarch64_link()
{
    # 修复路径
    sudo rm -rf /usr/lib/aarch64-linux-gnu /usr/include/aarch64-linux-gnu
    sudo ln -sf $SYSROOT/usr/lib/aarch64-linux-gnu       /usr/lib/aarch64-linux-gnu
    sudo ln -sf $SYSROOT/usr/include/aarch64-linux-gnu  /usr/include/aarch64-linux-gnu
    # 修复链接路径
    cd $SYSROOT/usr/lib/aarch64-linux-gnu
    sudo ln -sf openblas-pthread/libblas.so    libblas.so
    sudo ln -sf openblas-pthread/libblas.so.3  libblas.so.3
    sudo ln -sf openblas-pthread/liblapack.so     liblapack.so
    sudo ln -sf openblas-pthread/liblapack.so.3   liblapack.so.3
    sudo ln -sf openblas-pthread/libopenblas.so     libopenblas.so
    sudo ln -sf openblas-pthread/libopenblas.so.0   libopenblas.so.0

    # 复制添加的
    prebuild_target="$environment_dir/aarch64_sysroot_2204/prebuild_target"
    if [ -d $prebuild_target ]; then
        cp -axr $prebuild_target/*  $SYSROOT/
    fi
}

function torch_env_2204_orin()
{
    local_dir=${current_pwd}/../local
    if [ ! -d $local_dir ]; then
        echo "local dir $local_dir not exist"
        return
    fi
    cd $local_dir

    # 安装dcona
    export PATH=/opt/miniconda3/bin/:$PATH
    which conda 1>/dev/null
    if [ $? -ne 0 ]; then
        echo "# install miniconda"
        wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh -O /tmp/Miniconda3-latest-Linux-aarch64.sh
        chmod +x /tmp/Miniconda3-latest-Linux-aarch64.sh
        sudo mkdir -p /opt/miniconda3 ; sudo chmod 777 /opt/miniconda3
        /tmp/Miniconda3-latest-Linux-aarch64.sh -p /opt/miniconda3 -b -u
    fi
    conda init bash
    conda config --set auto_activate_base false

    # 安装流程 https://hlrobots.feishu.cn/docx/CCw2doG2Ro0OpRxlrrdcVpQ5nXc

    env_name="foundationpose"

    echo "# create conda env"
    env_list=$(conda env list | grep $env_name)
    if [ -n "$env_list" ]; then
        echo "$env_name 环境已存在，无需创建。"
    else
        conda create -y -n $env_name python=3.10
        print_and_exit_if_ret $? $FUNCNAME "conda create"
    fi

    echo "# activate env"
    source activate base
    conda activate $env_name
    print_and_exit_if_ret $? $FUNCNAME "conda activate $env_name"
    if [ "$CONDA_DEFAULT_ENV"x != "$env_name"x ]; then
        echo "conda activate $env_name 失败。"
        exit 1
    fi
    echo "# python3 --version 以下操作在 conda $env_name 环境下执行"
    python3 --version

    echo "# install conda-bash-completion"
    conda list | grep conda-forge | grep -i conda-bash-completion
    if [ $? -ne 0 ]; then
        conda install -y -c conda-forge conda-bash-completion
        print_and_exit_if_ret $? $FUNCNAME "conda install conda-bash-completion"
    else
        echo "  conda-bash-completion 已安装。"
    fi

    echo "# install iopath fvcore iopath"
    conda install -y -c conda-forge -c iopath fvcore iopath

    echo "# install eigen"
    conda list | grep conda-forge | grep -i eigen | grep 3.4.0
    if [ $? -ne 0 ]; then
        conda install -y conda-forge::eigen=3.4.0
        print_and_exit_if_ret $? $FUNCNAME "conda install eigen"
    else
        echo "  eigen 已安装。"
    fi
    export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:/opt/miniconda3/pkgs/eigen-3.4.0-h2a328a1_0/"

    req_txt="/workspace/hl_robot_orin/local/requirements_orin_tiny_install.txt"
    echo "# install $req_txt"
    sudo python3 -m pip install -r $req_txt $pip_server  | grep -v "Requirement already satisfied"
    print_and_exit_if_ret $? $FUNCNAME "conda pip install $req_txt"

    echo "# install nvidia-dlls"
    pip show nvdiffrast --quiet
    if [ $? -ne 0 ]; then
        python -m pip install --quiet --no-cache-dir git+https://github.com/NVlabs/nvdiffrast.git
        print_and_exit_if_ret $? $FUNCNAME "conda pip install nvidia-dlls"
    else
        echo "  nvdiffrast has been installed."
    fi

    echo "# install torch"
    whl_dir="/workspace/hl_robot_orin/local/"
    sudo python3 -m pip install $whl_dir/torch*.whl $pip_server --force-reinstall  | grep -v "Requirement already satisfied"
    print_and_exit_if_ret $? $FUNCNAME "conda pip install local file torch"
    
    echo "# install pytorch3d"
    conda list | grep pytorch3d
    if [ $? -ne 0 ]; then
        export FORCE_CUDA=1 ; pip install "git+https://github.com/facebookresearch/pytorch3d.git"
        print_and_exit_if_ret $? $FUNCNAME "conda install local file pytorch3d"
    else
        echo "  pytorch3d has been installed."
    fi
    echo "# test pytorch3d"
    sudo python3 $whl_dir/test_pytorch3d.py
    print_and_exit_if_ret $? $FUNCNAME "conda install test pytorch3d"

    echo "# print info"
    conda info --env
    pip3 list | grep -i torch
}

function modify_rootfs_2204_aarch64_buildroot()
{
    source_file="$environment_dir/docker_files_2204/sources.list"
    if [ -f $source_file ]; then
        sudo cp  $source_file  $SYSROOT/etc/apt/sources.list
    fi

    source_ubuntu_file="$environment_dir/docker_files_2204/sources_ubuntu.list"
    if [ -f $source_ubuntu_file ]; then
        echo "# "
        # 如果执行编译出现 apt update 失败，可以尝试打开下面一行，使用ubuntu默认的源
        echo "# 如果执行编译出现 apt update 失败，可以尝试打开下面一行，使用ubuntu默认的源"
        echo "#    docker_env/environment/rootfs_aarch64_22_build.sh modify_rootfs_2204_aarch64_buildroot()"
        # sudo cp  $source_ubuntu_file  $SYSROOT/etc/apt/sources.list
    fi

    # source_l4t_file="$environment_dir/docker_files_2204/nvidia-l4t-apt-source.list"
    # if [ -f $source_l4t_file ]; then
    #     sudo cp  $source_l4t_file  $SYSROOT/etc/apt/sources.list.d/
    # fi

    resolv_file="$environment_dir/docker_files_2204/resolv.conf"
    if [ -f $resolv_file ]; then
        sudo cp  $resolv_file  $SYSROOT/etc/resolv.conf
    fi

    if [ $is_pc == "true" ]; then
        sudo cp /usr/bin/qemu-aarch64-static ${SYSROOT}/usr/bin/
        mount_mem_fs
    elif [ $is_orin == "true" ]; then
        echo "# copy nvidia-l4t-apt-source.list to /etc/apt/sources.list.d/"
        sudo cp ${current_pwd}/../share/nvidia-l4t-apt-source.list /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
    fi
	
    EXEC_ROOTFS "mkdir -p /tmp ; chmod 777 /tmp"
    EXEC_ROOTFS chmod 777 /var/cache/man -R
    EXEC_ROOTFS chmod g+s /var/cache/man -R
    EXEC_ROOTFS chmod 777 /dev/null -R



    ###添加新库测试,省时间




    ###return
    


    EXEC_ROOTFS "apt update --list-cleanup || true"
    EXEC_ROOTFS "apt --fix-broken install -y || true"

    EXEC_ROOTFS apt install -y curl wget apt-utils apt-file sshpass gnupg gpgv gpgv2 gpgv1 
    EXEC_ROOTFS apt install -y python3-pip libyaml-cpp-dev libssl-dev sysstat ntpdate
    EXEC_ROOTFS apt install -y python3-numpy python3-dev
    EXEC_ROOTFS apt install -y libboost-system-dev libboost-filesystem-dev

    # realsense
    EXEC_ROOTFS "apt-key adv --keyserver hkp://pgp.mit.edu:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE"
    EXEC_ROOTFS "echo 'deb https://librealsense.intel.com/Debian/apt-repo jammy main' >> /etc/apt/sources.list"

    # ros2
    EXEC_ROOTFS curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    if [ "$is_x86"x = "true"x ]; then
        EXEC_ROOTFS "sed -i 's|https://mirrors.ustc.edu.cn/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' /etc/apt/sources.list"
    fi
    # EXEC_ROOTFS curl -sSL https://gitee.com/tyx6/rosdistro/raw/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    # EXEC_ROOTFS "echo 'deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://packages.ros.org/ros2/ubuntu jammy main' > /etc/apt/sources.list.d/ros2.list"
    EXEC_ROOTFS curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    EXEC_ROOTFS 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.ustc.edu.cn/ros2/ubuntu $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null'
    # EXEC_ROOTFS 'echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list'
    # EXEC_ROOTFS curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    # EXEC_ROOTFS "echo 'deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main' > /etc/apt/sources.list.d/ros2.list"

    

    EXEC_ROOTFS "apt update || true"
    EXEC_ROOTFS apt install -y libpython3.10-dev ament-cmake ros-humble-desktop
    EXEC_ROOTFS apt install -y ros-humble-tf2 ros-humble-tf2-tools ros-humble-tf2-msgs ros-humble-tf2-ros ros-humble-tf2-py
    EXEC_ROOTFS apt install -y ros-humble-nav-msgs
    EXEC_ROOTFS apt install -y ros-humble-diagnostic-updater ros-humble-rosidl-default-generators
    EXEC_ROOTFS apt install -y ros-humble-sensor-msgs ros-humble-sensor-msgs-py
    EXEC_ROOTFS apt install -y ros-humble-behaviortree-cpp
    EXEC_ROOTFS apt install -y ros-humble-compressed-image-transport ros-humble-compressed-depth-image-transport ros-humble-rosbag2-compression
    EXEC_ROOTFS apt install -y ros-humble-rsl
    EXEC_ROOTFS apt install -y ros-humble-backward-ros
    EXEC_ROOTFS apt install -y ros-humble-camera-info-manager
    EXEC_ROOTFS apt install -y ros-humble-image-publisher
    EXEC_ROOTFS apt install -y pkg-config
    EXEC_ROOTFS apt install -y libgstreamer1.0-dev 
    EXEC_ROOTFS apt install -y libgstreamer-plugins-base1.0-dev 
    EXEC_ROOTFS apt install -y gstreamer1.0-tools
    EXEC_ROOTFS apt install -y libglib2.0-dev  
    # EXEC_ROOTFS apt install -y ros-humble-pinocchio   # 不用直接安装，使用 aarch64_prebuild/install_2204_pinocchio_aarch64.tar.bz2


    # 使用编译支持cuda的
    # EXEC_ROOTFS apt install -y librealsense2-dev librealsense2-gl-dev librealsense2-utils  # librealsense2-dkms 
    
    EXEC_ROOTFS apt install -y ffmpeg libavcodec-dev libavutil-dev libswscale-dev libavformat-dev libavdevice-dev libglfw3-dev
    EXEC_ROOTFS apt install -y libgstreamer1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-opencv1.0-0 gstreamer1.0-rtsp libgstreamer-plugins-base1.0-dev
    
    EXEC_ROOTFS apt install -y liblcm-dev nlohmann-json3-dev

    EXEC_ROOTFS apt install -y libevdev-dev

    # pip
    EXEC_ROOTFS pip3 install pyzmq jinja2 typeguard $pip_server

    # 操作仓库
    EXEC_ROOTFS apt install -y portaudio19-dev libasound2
    # torch 在上面的 torch_env_2204_orin() conda 中安装
    # EXEC_ROOTFS pip3 install torch torchvision torchaudio $pip_server
    if [ $is_orin == "true" ]; then
        EXEC_ROOTFS pip3 install modelscope websocket simpleaudio grpcio pyaudio $pip_server
        EXEC_ROOTFS pip3 install funasr modelscope huggingface huggingface_hub pydub $pip_server
        EXEC_ROOTFS pip3 install numpy==1.24.4 $pip_server
        EXEC_ROOTFS pip3 install aiohttp $pip_server
        EXEC_ROOTFS pip3 install requests_toolbelt $pip_server
        EXEC_ROOTFS pip3 install janus $pip_server
    fi
    

    # pinocchio v3.2.0
    EXEC_ROOTFS apt install -y libsdformat-dev

    # nav
    EXEC_ROOTFS apt install -y ros-humble-pcl-ros ros-humble-imu-tools

    # jetpack
    if [ $is_orin == "true" ]; then
        echo "# install jetpack 需要等待较长的时间"
        EXEC_ROOTFS apt install -y nvidia-jetpack
    fi

    if [ $is_pc == "true" ]; then
	    umount_mem_fs
    fi
}

function main_modify_rootfs_2204_aarch64()
{
    # 判断是否已经执行过，修改上面的安装命令 modify_rootfs_2204_aarch64_buildroot，
    # 需要同步修改 rootfs_modify_ver 北京时间戳，时间戳格式不要改变
    flag_file=$1
    rootfs_modify_ver="2025-06-11 17:03:22"
    rootfs_modify_ts=$(date -d "$rootfs_modify_ver" +%s)

    stored_ver=$(cat $flag_file 2>/dev/null)
    if [ "$stored_ver"x != ""x ]; then
        stored_ts=$(date -d "$stored_ver" +%s 2>/dev/null)
    fi

    if [ ! -z "$stored_ts" ] && [ $stored_ts -ge $rootfs_modify_ts ]; then
        echo "# main_modify_rootfs_2204_aarch64() $stored_ts >= $rootfs_modify_ts  ($stored_ver >= $rootfs_modify_ver) , do nothing"
        modify_rootfs_2204_aarch64_link
        return 0
    fi

    echo "# main_modify_rootfs_2204_aarch64() 正在构建ORIN文件系统, 第一次可能花费时间较长..."
    # return   # 如果你不需要构建文件系统，可以打开这行，直接返回
    
    modify_rootfs_2204_aarch64_buildroot
    modify_rootfs_2204_aarch64_link

    EXEC_ROOTFS apt install -y sshpass
    echo "# count_success:$count_success  count_fail:$count_fail"
    if [ $count_fail -gt 0 ]; then
        echo "# main_modify_rootfs_2204_aarch64() count_fail:$count_fail 构建ORIN文件系统有失败的模块, 程序退出，可能是网络不稳定，请重试..."
        exit 1
    fi

    echo -n "$rootfs_modify_ver"  > $flag_file
}

argv_1=$1

# orin 端更新文件系统
if [ $is_orin == "true" ]; then
    current_pwd=$(cd $(dirname $0); pwd)
    if [ "$argv_1"x == "torch"x ]; then
        # 只安装torch
        torch_env_2204_orin
    else
        modify_rootfs_2204_aarch64_buildroot
    fi
    exit 0
elif [ "$argv_1"x == "rootfs_x86"x ]; then
    # 工控机环境, 上面的参数不能使用x86，会和build.sh冲突
    is_pc=false
    is_x86=true
    current_pwd=$(cd $(dirname $0); pwd)
    modify_rootfs_2204_aarch64_buildroot
fi
