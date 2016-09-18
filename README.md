# ElasticFusion #

Real-time dense visual SLAM system capable of capturing comprehensive dense globally consistent surfel-based maps of room scale environments explored using an RGB-D camera.

# Related Publications #
Please cite this work if you make use of our system in any of your own endeavors:

* **[ElasticFusion: Dense SLAM Without A Pose Graph](http://thomaswhelan.ie/Whelan15rss.pdf)**, *T. Whelan, S. Leutenegger, R. F. Salas-Moreno, B. Glocker and A. J. Davison*, RSS '15

# 1. What do I need to build it? #
* Ubuntu 14.04, 15.04 or 16.04 (Though many other linux distros will work fine)
* CMake
* OpenGL
* [CUDA >= 7.0](https://developer.nvidia.com/cuda-downloads)
* [OpenNI2](https://github.com/occipital/OpenNI2)
* SuiteSparse
* Eigen
* zlib
* libjpeg
* [Pangolin](https://github.com/stevenlovegrove/Pangolin)

Firstly, add [nVidia's official CUDA repository](https://developer.nvidia.com/cuda-downloads) to your apt sources, then run the following command to pull in most dependencies from the official repos:

```bash
sudo apt-get install -y cmake-qt-gui git build-essential libusb-1.0-0-dev libudev-dev openjdk-7-jdk freeglut3-dev libglew-dev cuda-7-5 libsuitesparse-dev libeigen3-dev zlib1g-dev libjpeg-dev
```

Afterwards install [OpenNI2](https://github.com/occipital/OpenNI2) and [Pangolin](https://github.com/stevenlovegrove/Pangolin) from source. Note, you may need to manually tell CMake where OpenNI2 is since Occipital's fork does not have an install option. It is important to build Pangolin last so that it can find some of the libraries it has optional dependencies on. 

When you have all of the dependencies installed, build the Core followed by the GUI. 

# 2. Is there an easier way to build it? #
Yes, if you run the *build.sh* script on a fresh clean install of Ubuntu 14.04 or 15.04, enter your password for sudo a few times and wait a few minutes all dependencies will get downloaded and installed and it should build everything correctly. This has not been tested on anything but fresh installs, so I would advise using it with caution if you already have some of the dependencies installed. It might also work on 16.04, but not guaranteed (modify it to coerce it into the 15.04 path).

# 3. Installation issues #

***`#include <Eigen/Core>` not found***

```bash
sudo ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen
sudo ln -sf /usr/include/eigen3/unsupported /usr/include/unsupported
```

***invalid use of incomplete type ‘const struct Eigen ...***

Pangolin must be installed AFTER all the other libraries to make use of optional dependencies.

***GLSL 3.30 is not supported. Supported versions are 1.10, 1.20, 1.30, 1.00 ES and 3.00 ES***

Make sure you are running ElasticFusion on your nVidia GPU. In particular, if you have an Optimus GPU
- If you use Prime, follow instructions [here](http://askubuntu.com/questions/661922/how-am-i-supposed-to-use-nvidia-prime)
- If you use Bumblebee, remember to run as `optirun ./ElasticFusion`

# 4. How do I use it? #
There are three subprojects in the repo:

* The *Core* is the main engine which builds into a shared library that you can link into other projects and treat like an API. 
* The *GUI* is the graphical interface used to run the system on either live sensor data or a logged data file. 
* The *GPUTest* is a small benchmarking program you can use to tune the CUDA kernel launch parameters used in the main engine. 

The GUI (*ElasticFusion*) can take a bunch of parameters when launching it from the command line. They are as follows:

* *-cal <calibration>* : Loads a camera calibration file specified as *fx fy cx cy*.
* *-l <logfile>* : Processes the specified .klg log file.
* *-p <poses>* : Loads ground truth poses to use instead of estimated pose.
* *-c <confidence>* : Surfel confidence threshold (default *10*).
* *-d <depth>* : Cutoff distance for depth processing (default *3*m).
* *-i <icp>* : Relative ICP/RGB tracking weight (default *10*).
* *-ie <error>* : Local loop closure residual threshold (default *5e-05*).
* *-ic <count>* : Local loop closure inlier threshold (default *35000*).
* *-cv <covariance>* : Local loop closure covariance threshold (default *1e-05*).
* *-pt <photometric>* : Global loop closure photometric threshold (default *115*).
* *-ft <threshold>* : Fern encoding threshold (default *0.3095*).
* *-t <time>* : Time window length (default *200*).
* *-s <skip>* : Frames to skip at start of log.
* *-e <end>* : Cut off frame of log.
* *-f* : Flip RGB/BGR.
* *-icl* : Enable this if using the [ICL-NUIM](http://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html) dataset (flips normals to account for negative focal length on that data).
* *-o* : Open loop mode.
* *-rl* : Enable relocalisation.
* *-fs* : Frame skip if processing a log to simulate real-time.
* *-q* : Quit when finished a log.
* *-fo* : Fast odometry (single level pyramid).
* *-nso* : Disables SO(3) pre-alignment in tracking.
* *-r* : Rewind and loop log forever. 
* *-ftf* : Do frame-to-frame RGB tracking. 
* *-sc* : Showcase mode (minimal GUI).

Essentially by default *./ElasticFusion* will try run off an attached ASUS sensor live. You can provide a .klg log file instead with the -l parameter. You can capture .klg format logs using either [Logger1](https://github.com/mp3guy/Logger1) or [Logger2](https://github.com/mp3guy/Logger2). 

# 5. How do I just use the Core API? #
The libefusion.so shared library which gets built by the Core is what you want to link against.

An example of this can be seen in the GUI code. Essentially all you need to do is utilise the provided Findefusion.cmake file in GUI/src and include the following in your CMakeLists.txt file:

    find_package(efusion REQUIRED)
    include_directories(${EFUSION_INCLUDE_DIR})
    target_link_libraries(MyProject ${EFUSION_LIBRARY})
    
To then use the Core API, make sure to include the header file in your source file:
```cpp
    #include <ElasticFusion.h>
```

Initialise the static configuration parameters once somewhere at the start of your program (this [smells](http://en.wikipedia.org/wiki/Code_smell), but whatever):
```cpp
    Resolution::getInstance(640, 480);
    Intrinsics::getInstance(528, 528, 320, 240);
```

Make an ElasticFusion object and start using it:
```cpp
    ElasticFusion eFusion;
    eFusion.processFrame(rgb, depth, timestamp, currentPose, weightMultiplier);
```

See the source code of MainController.cpp in the GUI source to see more usage.

# 6. Datasets #

We have provided a sample dataset which you can run easily with ElasticFusion for download [here](http://www.doc.ic.ac.uk/~sleutene/datasets/elasticfusion/dyson_lab.klg). Launch it as follows:

```bash
./ElasticFusion -l dyson_lab.klg
```

# 7. License #
ElasticFusion is freely available for non-commercial use only.  Full terms and conditions which govern its use are detailed [here](http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/) and in the LICENSE.txt file.

# 8. FAQ #
***What are the hardware requirements?***

A [very fast nVidia GPU (3.5TFLOPS+)](https://en.wikipedia.org/wiki/List_of_Nvidia_graphics_processing_units#GeForce_900_Series), and a fast CPU (something like an i7). If you want to use a non-nVidia GPU you can rewrite the tracking code or substitute it with something else, as the rest of the pipeline is actually written in the OpenGL Shading Language. 

***How can I get performance statistics?***

Download [Stopwatch](https://github.com/mp3guy/Stopwatch) and run *StopwatchViewer* at the same time as ElasticFusion. 

***I ran a large dataset and got assert(graph.size() / 16 < MAX_NODES) failed***

Currently there's a limit on the number of nodes in the deformation graph down to lazy coding (using a really wide texture instead of a proper 2D one). So we're bound by the maximum dimension of a texture, which is 16384 on modern cards/OpenGL. Either fix the code so this isn't a problem any more, or increase the modulo factor in *Shaders/sample.geom*. 

***I have a nice new laptop with a good GPU but it's still slow***

If your laptop is running on battery power the GPU will throttle down to save power, so that's unlikely to work (as an aside, [Kintinuous](https://github.com/mp3guy/Kintinuous) will run at 30Hz on a modern laptop on battery power these days). You can try disabling SO(3) pre-alignment, enabling fast odometry, only using either ICP or RGB tracking and not both, running in open loop mode or disabling the tracking pyramid. All of these will cost you accuracy. 

***I saved a map, how can I view it?***

Download [Meshlab](http://meshlab.sourceforge.net/). Select Render->Shaders->Splatting. 

***The map keeps getting corrupted - tracking is failing - loop closures are incorrect/not working***

Firstly, if you're running live and not processing a log file, ensure you're hitting 30Hz, this is important. Secondly, you cannot move the sensor extremely fast because this violates the assumption behind projective data association. In addition to this, you're probably using a primesense, which means you're suffering from motion blur, unsynchronised cameras and rolling shutter. All of these are aggravated by fast motion and hinder tracking performance. 

If you're not getting loop closures and expecting some, pay attention to the inlier and residual graphs in the bottom right, these are an indicator of how close you are to a local loop closure. For global loop closures, you're depending on [fern keyframe encoding](http://www.doc.ic.ac.uk/~bglocker/pdfs/glocker2015tvcg.pdf) to save you, which like all appearance-based place recognition methods, has its limitations. 

***Is there a ROS bridge/node?***

No. The system relies on an extremely fast and tight coupling between the mapping and tracking on the GPU, which I don't believe ROS supports natively in terms of message passing. 

***This doesn't seem to work like it did in the videos/papers***

A substantial amount of refactoring was carried out in order to open source this system, including rewriting a lot of functionality to avoid certain licenses and reduce dependencies. Although great care was taken during this process, it is possible that performance regressions were introduced and have not yet been discovered.
