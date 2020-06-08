# mySLAM

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

##### libglog.a and libglog.a are two files installed in /usr/lib to make find_package() enable to find Glog.
when installing glog, it shows when it is without sudo:  
CMake Error at cmake_install.cmake:36 (file):  
  file INSTALL cannot copy file "/home/nipnie/glog/build/libglog.a" to  
  "/usr/local/lib/libglog.a".  
  
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
