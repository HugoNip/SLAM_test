# mySLAM

### gtest package
#### Issue
build.sh: Could NOT find GTest (missing: GTEST_LIBRARY GTEST_MAIN_LIBRARY) #571


```
sudo apt-get install cmake libgtest-dev
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
 
# copy or symlink libgtest.a and libgtest_main.a to your /usr/lib folder
sudo cp *.a /usr/lib
```
