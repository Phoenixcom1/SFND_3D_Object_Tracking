# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

## Performance evaluation 1

_Find examples where the TTC estimate of the Lidar sensor does not seem plausible. Describe your observations and provide a sound argumentation why you think this happened._

Initial assumption is that the ego vehicle is continuously getting closer to the vehicle in front (resulting in monotonously decreasing TTC). Within the results it can be observed, that e.g. between frames 2/3 and 3/4, the Lidar based TTC is increasing instead of decreasing.
This is most probably caused by lidar points associated with the bounding box / ROI but not been part of the proceeding vehicle (outliers). For lowering the impact of such outliers, the average filter got replaced by an median filter. Though, results still show an impact by such outliers.
As a next step an improved outlier detection should be introduced.


## Performance evaluation 2

_Run several detector / descriptor combinations and look at the differences in TTC estimation. Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. As with Lidar, describe your observations again and also look into potential reasons._

The camera based TTC calculation is directly depending on the performance of the detector/descriptor and matcher combination. For a reliable TTC calculation a sufficient amount of keypoints and correct matches is essential.
In the following good and bad examples are shown. 

### Good examples
Detector Type | Descriptor Type | Matcher Type | Selector Type |
--|--|--|--|
SHITOMASI | BRISK | MAT_BF | SEL_NN |

<img src="images/SHITOMASI_BRISK_NN.png" width="752" height="449" />

Detector Type | Descriptor Type | Matcher Type | Selector Type |
--|--|--|--|
AKAZE | BRISK | MAT_BF | SEL_NN |

<img src="images/AKAZE_BRISK_NN.png" width="752" height="449" />

Detector Type | Descriptor Type | Matcher Type | Selector Type |
--|--|--|--|
SIFT | FREAK | MAT_BF | SEL_NN |

<img src="images/SIFT_FREAK_NN.png" width="752" height="449" />

### Bad example

Within the results it can be observed, that due to few keypoints provided by the HARRIS detector, the TTC calculation is unreliable and misleading. An example is given below.

Detector Type | Descriptor Type | Matcher Type | Selector Type |
--|--|--|--|
HARRIS | SIFT | MAT_BF | SEL_NN |

<img src="images/HARRIS_SIFT_NN.png" width="752" height="449" />