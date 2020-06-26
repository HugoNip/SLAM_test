# Introduction
This project shows a example of SLAM.

# Results
### Point cloud map
![Screenshot%20from%202020-06-25%2021-37-02.png](https://github.com/HugoNip/mySLAM/blob/master/figures/Screenshot%20from%202020-06-25%2021-37-02.png)

### Keypoint (green color)
![Screenshot%20from%202020-06-25%2021-37-55.png](https://github.com/HugoNip/mySLAM/blob/master/figures/Screenshot%20from%202020-06-25%2021-37-55.png)

### Data
![Screenshot%20from%202020-06-25%2021-38-42.png](https://github.com/HugoNip/mySLAM/blob/master/figures/Screenshot%20from%202020-06-25%2021-38-42.png)

# Required Packages
### Glog Package
#### Source
https://github.com/google/glog/blob/master/cmake/INSTALL.md

#### Building Glog with CMake
When building Glog as a standalone project, on Unix-like systems with GNU Make as build tool, the typical workflow is:   

1. Get the source code and change to it. e.g. cloning with git:
```
git clone git@github.com:google/glog.git
cd glog
```
2. Run CMake to configure the build tree.
```
cmake -H. -Bbuild -G "Unix Makefiles"
```
note: To get the list of available generators (e.g. Visual Studio), use -G ""

3. Afterwards, generated files can be used to compile the project.
```
cmake --build build
```
4. Test the build software.
```
cmake --build build --target test
```
5. Install the built files.
```
cmake --build build --target install
```

#### Issue: Failed to find glog
CMake Error at cmake/FindGlog.cmake:77 (MESSAGE):
Failed to find glog - Could not find glog include directory, set
GLOG_INCLUDE_DIR to directory containing glog/logging.h
Call Stack (most recent call first):
cmake/FindGlog.cmake:103 (GLOG_REPORT_NOT_FOUND)
CMakeLists.txt:27 (FIND_PACKAGE)
  
#### The solution is to reinstall glog and rebuid it.



### gtest package
#### Issue: Could NOT find GTest (missing: GTEST_LIBRARY GTEST_MAIN_LIBRARY) #571
##### I find gtest in /usr/src, but it seems that it is not built and compiled, so I build and compilte gtest.

```
sudo apt-get install cmake libgtest-dev
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
```

##### Then, copy or symlink libgtest.a and libgtest_main.a to your /usr/lib folder
```
sudo cp *.a /usr/lib
```

### Gflags Package
#### Source
https://github.com/gflags/gflags
#### Build and compile
```
mkdir build
cd build
cmake ..
make
sudo make install
```

## Run
```
./bin/run_kitti_stereo
./bin/test_triangulation
```


## Reference
[Source](https://github.com/HugoNip/slambook2/tree/master/ch13)

