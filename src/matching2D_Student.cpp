
#include <numeric>
#include "matching2D.hpp"

using namespace std;

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis) {

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    auto t = (double)cv::getTickCount();

    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    //perform a non-maximum suppression (NMS) in a local neighborhood around each maximum
    const float maxAllowedOverlap = 0.0;

    for (int i = 0; i < dst_norm.cols; i++) {
        for (int j = 0; j < dst_norm.rows; j++) {
            cv::Point2f curr_pt(i, j);

            const int curr_response = (int) dst_norm.at<float>(curr_pt);
            //https://bitbashing.io/comparing-floats.html
            if (curr_response >= minResponse) {

                cv::KeyPoint newKeyPoint(curr_pt, 2 * apertureSize, -1, curr_response);
                bool overlapping = false;
                for (cv::KeyPoint kpt : keypoints) {
                    float overlap = cv::KeyPoint::overlap(newKeyPoint, kpt);
                    if (maxAllowedOverlap < overlap) {
                        overlapping = true;
                        if (curr_response > kpt.response) {
                            kpt = newKeyPoint;
                        }
                    }
                }

                if (!overlapping) {
                    keypoints.push_back(newKeyPoint);
                }
            }
        }
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(dst_norm_scaled, keypoints, visImage, cv::Scalar::all(-1),
                          cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 5);
        cv::imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    auto t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto & corner : corners)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f(corner.x, corner.y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsFAST(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    int threshold = 10;
    bool nonmaxSuppression = true;
    cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16;

    cv::Ptr<cv::FeatureDetector> FASTdetector = cv::FastFeatureDetector::create(threshold, nonmaxSuppression, type);

    auto t2 = (double)cv::getTickCount();
    FASTdetector->detect(img, keypoints);
    t2 = ((double)cv::getTickCount() - t2) / cv::getTickFrequency();
    cout << "FAST detection with n= " << keypoints.size() << " keypoints in " << 1000 * t2 / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "FAST Corner Detector Results";
        cv::namedWindow(windowName, 1);
        imshow(windowName, visImage);
        cv::waitKey(0); // wait for keyboard input before continuing
    }

}

void detKeypointsBRISK(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    int Threshl=30;
    int Octaves=3;
    float PatternScales=1.0f;

    cv::Ptr<cv::FeatureDetector> BRISKdetector = cv::BRISK::create(Threshl, Octaves, PatternScales);

    auto t2 = (double)cv::getTickCount();
    BRISKdetector->detect(img, keypoints);
    t2 = ((double)cv::getTickCount() - t2) / cv::getTickFrequency();
    cout << "BRISK detection with n= " << keypoints.size() << " keypoints in " << 1000 * t2 / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "BRISK Corner Detector Results";
        cv::namedWindow(windowName, 1);
        imshow(windowName, visImage);
        cv::waitKey(0); // wait for keyboard input before continuing
    }
}

void detKeypointsORB(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    int nfeatures=500;
    float scaleFactor=1.2f;
    int nlevels=8;
    int edgeThreshold=31;
    int firstLevel=0;
    int WTA_K=2;
    cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE;
    int patchSize=31;
    int fastThreshold=20;

    cv::Ptr<cv::FeatureDetector> ORBdetector = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);

    auto t2 = (double)cv::getTickCount();
    ORBdetector->detect(img, keypoints);
    t2 = ((double)cv::getTickCount() - t2) / cv::getTickFrequency();
    cout << "ORB detection with n= " << keypoints.size() << " keypoints in " << 1000 * t2 / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "ORB Corner Detector Results";
        cv::namedWindow(windowName, 1);
        imshow(windowName, visImage);
        cv::waitKey(0); // wait for keyboard input before continuing
    }
}

void detKeypointsAKAZE(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    cv::AKAZE::DescriptorType descriptor_type = cv::AKAZE::DESCRIPTOR_MLDB;
    int descriptor_size = 0;
    int descriptor_channels = 3;
    float threshold = 0.001f;
    int nOctaves = 4;
    int nOctaveLayers = 4;
    cv::KAZE::DiffusivityType diffusivity = cv::KAZE::DIFF_PM_G2;

    cv::Ptr<cv::FeatureDetector> AKAZEdetector = cv::AKAZE::create(descriptor_type, descriptor_size, descriptor_channels, threshold, nOctaves, nOctaveLayers, diffusivity);

    auto t2 = (double)cv::getTickCount();
    AKAZEdetector->detect(img, keypoints);
    t2 = ((double)cv::getTickCount() - t2) / cv::getTickFrequency();
    cout << "AKAZE detection with n= " << keypoints.size() << " keypoints in " << 1000 * t2 / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "AKAZE Corner Detector Results";
        cv::namedWindow(windowName, 1);
        imshow(windowName, visImage);
        cv::waitKey(0); // wait for keyboard input before continuing
    }
}

void detKeypointsSIFT(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SIFT::create();

    auto t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "SIFT detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "SIFT Corner Detector Results";
        cv::namedWindow(windowName, 1);
        imshow(windowName, visImage);
        cv::waitKey(0); // wait for keyboard input before continuing
    }
}

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, const std::string& descriptorType, std::string& matcherType, std::string& selectorType, std::string& descriptorName)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType == "MAT_BF")
    {
        int normType = descriptorType == "DES_BINARY"? cv::NORM_HAMMING : cv::NORM_L2;
        if(descriptorName=="SIFT")
        {
            cout << "SIFT is incompatible with hamming distance" << endl;
            normType = cv::NORM_L2;
        }

        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType == "MAT_FLANN")
    {
        matcher = cv::FlannBasedMatcher::create();
        if(descSource.type()!=CV_32F)
        {
            descSource.convertTo(descSource, CV_32F);
        }
        if(descRef.type()!=CV_32F)
        {
            descRef.convertTo(descRef, CV_32F);
        }
    }

    // perform matching task
    if (selectorType == "SEL_NN")
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType == "SEL_KNN")
    { // k nearest neighbors (k=2)

        //k-nearest-neighbor matching
        vector<vector<cv::DMatch>> kMatches;
        auto t = (double)cv::getTickCount();
        matcher->knnMatch(descSource, descRef, kMatches, 2); // Finds the k best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (KNN) matcher with n=" << kMatches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;

        //filter matches using descriptor distance ratio test
        float distThreshould = 0.8;
        for(vector<cv::DMatch> kMatch : kMatches)
        {
            if(kMatch[0].distance < distThreshould * kMatch[1].distance)
            {
                matches.push_back(kMatch[0]);
            }
        }
//        cout << kMatches.size() << " KNN matches" << endl;
//        cout << kMatches.size() - matches.size() << " matches removed by descriptor distance ratio test" << endl;
//        cout << matches.size() << " matches left" << endl;
//        cout << (((double)kMatches.size() - (double)matches.size())/(double)kMatches.size())*100.0 << " % removed" << endl;

    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType == "BRISK")
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if(descriptorType == "BRIEF")
    {
        int bytes = 32;
        bool use_orientation = false;

        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(bytes, use_orientation);
    }
    else if(descriptorType == "ORB")
    {
        int nfeatures=500;
        float scaleFactor=1.2f;
        int nlevels=8;
        int edgeThreshold=31;
        int firstLevel=0;
        int WTA_K=2;
        cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE;
        int patchSize=31;
        int fastThreshold=20;

        extractor = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);
    }
    else if(descriptorType == "FREAK")
    {
        bool orientationNormalized = true;
        bool scaleNormalized = true;
        float patternScale = 22.0f;
        int nOctaves = 4;
        const std::vector<int>& selectedPairs = std::vector<int>();

        extractor = cv::xfeatures2d::FREAK::create(orientationNormalized, scaleNormalized, patternScale, nOctaves, selectedPairs);
    }
    else if(descriptorType == "AKAZE")
    {
        cv::AKAZE::DescriptorType descriptor_type = cv::AKAZE::DESCRIPTOR_MLDB;
        int descriptor_size = 0;
        int descriptor_channels = 3;
        float threshold = 0.001f;
        int nOctaves = 4;
        int nOctaveLayers = 4;
        cv::KAZE::DiffusivityType diffusivity = cv::KAZE::DIFF_PM_G2;

        extractor = cv::AKAZE::create(descriptor_type, descriptor_size, descriptor_channels, threshold, nOctaves, nOctaveLayers, diffusivity);
    }
    else if(descriptorType == "SIFT")
    {
        //TODO cover type issue CV_8U/32S
        extractor = cv::xfeatures2d::SIFT::create();
    }

    // perform feature description
    auto t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}