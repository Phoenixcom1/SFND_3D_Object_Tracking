
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unordered_set>



#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{

    std::vector<cv::DMatch>  kptMatchesInROI;
    double avDistOfkptMachtesInROI;
    //Looping through key point matches while checking if the key point "on the current side of the match" is located within the bounding box (previous = query, current = train)
    for(auto match : kptMatches)
    {
        auto kpt = kptsCurr.at(match.trainIdx);
        if(boundingBox.roi.contains(kpt.pt))
        {
            kptMatchesInROI.push_back(match);
            avDistOfkptMachtesInROI += match.distance;
        }
    }

    if (kptMatchesInROI.size() > 0)
    {
        avDistOfkptMachtesInROI = avDistOfkptMachtesInROI/kptMatchesInROI.size();
    }
    else return;

    double threshould = avDistOfkptMachtesInROI * 0.8;

    for(auto match : kptMatchesInROI)
    {
        if(match.distance < threshould)
        {
            boundingBox.kptMatches.push_back(match);
        }
    }

    std::cout << "No of Key Point Matches: " << boundingBox.kptMatches.size()   << std::endl;


}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios
    double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();


    sort(distRatios.begin(), distRatios.end());
    double medianDistRatio;
    // check for un-/even case
    if (distRatios.size() % 2 != 0)
    {
        //uneven result will be casted -> 7/2 = 3.5 -> 3
        medianDistRatio = distRatios[distRatios.size()/2];
    }
    else
    {
        medianDistRatio = distRatios[((distRatios.size()-1)/2 + distRatios.size()/2) /2];
    }

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medianDistRatio);

}

bool sortLidarPoint(LidarPoint pt1, LidarPoint pt2){ return pt1.x < pt2.x;}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{

    double dT = 1/frameRate;    // time between two measurements in seconds
    double laneWidth = 2.0;     // assumed width of the ego lane

    vector<double> prevDataVector;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if (abs(it->y) <= laneWidth / 2.0)
        {
            prevDataVector.push_back(it->x);
        }
    }

    vector<double> currDataVector;
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        if (abs(it->y) <= laneWidth / 2.0)
        {
            currDataVector.push_back(it->x);
        }
    }

    double meanMinXPrev = std::accumulate(prevDataVector.begin(), prevDataVector.end(), 0.0) / prevDataVector.size();
    double meanMinXCurr = std::accumulate(currDataVector.begin(), currDataVector.end(), 0.0) / currDataVector.size();

    TTC = meanMinXCurr * dT / std::abs(meanMinXPrev - meanMinXCurr);
    //return;

    size_t size = prevDataVector.size();
    sort(prevDataVector.begin(), prevDataVector.end());
    double medianPrevData = 0;
    if (size % 2 == 0)
    {
        medianPrevData = (prevDataVector[size / 2 - 1] + prevDataVector[size / 2]) / 2;
    }
    else
    {
        medianPrevData = prevDataVector[size / 2];
    }

    size = currDataVector.size();
    sort(currDataVector.begin(), currDataVector.end());
    double medianCurrData = 0;
    if (size % 2 == 0)
    {
        medianCurrData = (currDataVector[size / 2 - 1] + currDataVector[size / 2]) / 2;
    }
    else
    {
        medianCurrData = currDataVector[size / 2];
    }

    if ((prevDataVector.size() == 0) || (currDataVector.size() == 0))
    {
        TTC = NAN;
        return;
    }

    // compute TTC from both measurements
    TTC = medianCurrData * dT / (medianPrevData - medianCurrData);
    return;

}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{

    int bbCount_prev = prevFrame.boundingBoxes.size();
    int bbCount_curr = currFrame.boundingBoxes.size();
    //Matrix of bBoxes prev x bBoxes curr and number of matched key points between as value
    int interBBmatchingCounts[bbCount_prev][bbCount_curr];

    for ( int i = 0; i < prevFrame.boundingBoxes.size(); i++ )
    {
        for ( int j = 0; j < currFrame.boundingBoxes.size(); j++ )
        {
            interBBmatchingCounts[i][j] = 0;
        }
    }

    //Iterating through all key point matches between the two images
    for(auto match : matches)
    {
        //Extracting the corresponding bounding box IDs the key points are belonging to within the individual frames
        cv::KeyPoint kpt_prev = prevFrame.keypoints[match.queryIdx];
        cv::KeyPoint kpt_curr = currFrame.keypoints[match.trainIdx];

        std::vector<int> bbIDs_prev, bbIDs_curr;

        //Getting associated bounding boxes for matched key point in previous frame
        for(const auto& bBox : prevFrame.boundingBoxes)
        {
            if(bBox.roi.contains(kpt_prev.pt))
            {
                bbIDs_prev.push_back(bBox.boxID);
            }
        }
        //Getting associated bounding boxes for matched key point in current frame
        for(const auto& bBox : currFrame.boundingBoxes)
        {
            if(bBox.roi.contains(kpt_curr.pt))
            {
                bbIDs_curr.push_back(bBox.boxID);
            }
        }
        //Counting number of matches between all possible bBox combinations between the frames to find the highest one
        if (!bbIDs_prev.empty() && !bbIDs_curr.empty())
        {
            for (auto bbID_prev : bbIDs_prev)
            {
                for (auto bbID_curr : bbIDs_curr)
                {
                    //increasing match counter between id_prev and id_curr
                    interBBmatchingCounts[bbID_prev][bbID_curr] += 1;
                }
            }
        }
    }




    //Finding bBox matches by checking every bBox in the previous frame for the highest inter bounding box matching count in the current frame.
    for (int i = 0; i < bbCount_prev; i++)
    {

        int count_max = 0;
        int id_max =0;

        for (int j = 0; j < bbCount_curr ; j++)
        {
            if (interBBmatchingCounts[i][j] > count_max)
            {
                count_max = interBBmatchingCounts[i][j];
                id_max = j;
            }
        }


        bbBestMatches[i] = id_max;
    }
}
