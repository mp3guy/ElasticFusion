#!/bin/bash


# source: https://gist.github.com/phatblat/1713458
# Save script's current directory
DIR=$(pwd)

#
# Check if Homebrew is installed
#
    OS=`uname`
    case $OS in
      'Linux')
        OS='Linux'
        # alias ls='ls --color=auto'
        # curl -fsSL https://raw.githubusercontent.com/ahundt/homebrew-robotics/master/linuxbrew.sh | bash /dev/stdin
        # export PKG_CONFIG_PATH="/usr/bin/pkg-config:$HOME/.linuxbrew/bin/pkg-config"
        # export PKG_CONFIG_LIBDIR="/usr/lib/pkgconfig:$HOME/.linuxbrew/lib/pkgconfig"
        # export PATH="$HOME/.linuxbrew/bin:$PATH"       

        mkdir deps &> /dev/null
        cd deps

        #Add necessary extra repos
        version=$(lsb_release -a 2>&1)
        if [[ $version == *"14.04"* ]] ; then
            wget http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1404/x86_64/cuda-repo-ubuntu1404_7.5-18_amd64.deb
            sudo dpkg -i cuda-repo-ubuntu1404_7.5-18_amd64.deb
            rm cuda-repo-ubuntu1404_7.5-18_amd64.deb
            sudo apt-get update
        elif [[ $version == *"15.04"* ]] ; then
            wget http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1504/x86_64/cuda-repo-ubuntu1504_7.5-18_amd64.deb
            sudo dpkg -i cuda-repo-ubuntu1504_7.5-18_amd64.deb
            rm cuda-repo-ubuntu1504_7.5-18_amd64.deb
            sudo apt-get update
        else
            echo "Don't use this on anything except 14.04 or 15.04"
            exit
        fi

        sudo apt-get install -y cmake-qt-gui git build-essential libusb-1.0-0-dev libudev-dev openjdk-7-jdk freeglut3-dev libglew-dev cuda-7-5 libsuitesparse-dev libeigen3-dev zlib1g-dev libjpeg-dev

        #Installing Pangolin
        git clone https://github.com/stevenlovegrove/Pangolin.git
        cd Pangolin
        mkdir build
        cd build
        cmake ../ -DAVFORMAT_INCLUDE_DIR="" -DCPP11_NO_BOOST=ON
        make -j8
        cd ../..

        #Up to date OpenNI2
        git clone https://github.com/occipital/OpenNI2.git
        cd OpenNI2
        make -j8
        cd ..                        
        ;;
      'FreeBSD')
        OS='FreeBSD'
        alias ls='ls -G'
        ;;
      'WindowsNT')
        OS='Windows'
        ;;
      'Darwin') 
        OS='Mac'
        /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
        brew tap homebrew/science
        brew tap ahundt/robotics
        brew install openni2 pangolin
        ;;
      'SunOS')
        OS='Solaris'
        ;;
      'AIX') ;;
      *) ;;
    esac    
else
    brew update
fi


#Actually build ElasticFusion
mkdir build
cd build
cmake ..

#old style Actually build ElasticFusion
# cd Core
# mkdir build
# cd build
# cmake ../src
# make -j8
# cd ../../GPUTest
# mkdir build
# cd build
# cmake ../src
# make -j8
# cd ../../GUI
# mkdir build
# cd build
# cmake ../src
# make -j8
