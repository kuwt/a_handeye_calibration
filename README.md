**Purpose:**

This repo serves as a ros independent build of handeye calibration. It main purpose is to remove those dependencies that
https://github.com/jhu-lcsr/handeye_calib_camodocal requires, which are not really needed when we do not use ROS.
It only solves AX = XB and gives X, which should be the transformation from eye frame to robot flange frame.

![Image description](./doc/wdOyg.png)


**Currently supported:**
1. windows

**Dependencies**
1. Opencv3 (tested on opencv 3.2.0)
2. Eigen3
3. ceres-solver

**User Guide**
1. Build the exe with the dependencies
2. Input: transpair.yml; Explanation  T1: Robot pose or baseTee , T2:the transform from camera coordinate frame to AR code/chess board coordinate frame.
3. Input: config.json; you can find an example in ./example_resource
4. Output: eeTeye.yml, it is the transform from the eye frame to the robot flange frame