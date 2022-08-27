
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

using BoxIdx = int;
using KeyPointIdx = int;
using BoxIdPair = pair<BoxIdx, BoxIdx>;

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void
clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor,
                    cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT) {
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1) {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2) {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt)) {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1) {
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait) {
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for (auto it1 = boundingBoxes.begin(); it1 != boundingBoxes.end(); ++it1) {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
        float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2) {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin < xw ? xwmin : xw;
            ywmin = ywmin < yw ? ywmin : yw;
            ywmax = ywmax > yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top < y ? top : y;
            left = left < x ? left : x;
            bottom = bottom > y ? bottom : y;
            right = right > x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int) it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
        putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125), cv::FONT_ITALIC, 2, currColor);
    }

    // plot distance markers
    float lineSpacing = 0.5; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i) {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if (bWait) {
        cv::waitKey(0); // wait for key to be pressed
    }
}

inline double ttc(const double deltaT, const double d0, const double d1) {
    return d1 * deltaT / (d0 - d1);
}

template<typename A, typename AtoDouble>
inline double computeMedian(std::vector<A> &vec, AtoDouble projection) {
    std::sort(vec.begin(), vec.end(), [&](const A &p, const A &q) { return projection(p) < projection(q); });
    size_t idx = std::floor((vec.size() - 1) / 2);
    auto leftMedianValue = projection(vec[idx]);
    return vec.size() % 2 ? (leftMedianValue + projection(vec[idx + 1])) / 2 : leftMedianValue;
}

inline void inlierBounds(const std::vector<double> &matchDistances, double &mean, double &threshold) {
    auto N = (double) matchDistances.size();

    for (auto &distance: matchDistances) {
        mean += distance;
        threshold += distance * distance;
    }

    mean /= N;
    threshold -= (N * mean * mean);
    threshold = 3 * sqrt(threshold / (N - 1));
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, const std::vector<cv::KeyPoint> &kptsPrev,
                              const std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches) {

    // Fill the matches distances collection. The query image is the previous frame.
    std::vector<double> matchDistances(kptMatches.size());
    std::transform(kptMatches.begin(), kptMatches.end(), matchDistances.begin(), [&](const cv::DMatch &match) {
        return cv::norm(kptsCurr[match.trainIdx].pt - kptsPrev[match.queryIdx].pt);
    });

    // compute mean and standard deviation, then remove elements outliers.
    // outliers are 3 standard deviations away from the mean.
    double mean = 0, threshold = 0;
    inlierBounds(matchDistances, mean, threshold);

    // Set inliers matches; they are matches which have distances within 3 standard deviations
    // of the mean of all match distances.
    auto matchIt = kptMatches.cbegin();
    auto distIt = matchDistances.cbegin();

    for (; matchIt != kptMatches.cend(); ++matchIt, ++distIt) {
        if (std::abs(*distIt - mean) < threshold) {
            boundingBox.keypoints.push_back(kptsCurr[matchIt->trainIdx]);
            boundingBox.kptMatches.push_back(*matchIt);
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(const std::vector<cv::KeyPoint> &kptsPrev, const std::vector<cv::KeyPoint> &kptsCurr,
                      const std::vector<cv::DMatch> &kptMatches, const double frameRate, double &TTC, cv::Mat *visImg, const size_t frame) {

    std::vector<double> distanceRatios;
    double distPrev, distCurr, medianRatio;

    //std::cout << "| Frame " << frame << " | " << kptsCurr.size() << " | "  << kptMatches.size() << " |\n";

    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it) {
        for(auto jt = it+1; jt != kptMatches.end(); ++jt) {
            // the query image is the previous frame.
            distPrev = cv::norm(kptsPrev[jt->queryIdx].pt - kptsPrev[it->queryIdx].pt);
            distCurr = cv::norm(kptsCurr[jt->trainIdx].pt - kptsCurr[it->trainIdx].pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= 100) {
                distanceRatios.push_back(distCurr / distPrev);
            }
        }
    }

    medianRatio = computeMedian(distanceRatios, [](const double& p) { return p; });
    TTC = ttc(1.0 / frameRate, medianRatio, 1);
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, const double frameRate, double &TTC) {

    auto projection = [](const LidarPoint& p) { return p.x; };
    double d0 = computeMedian(lidarPointsPrev, projection);
    double d1 = computeMedian(lidarPointsCurr, projection);

    // std:cout << "| " << d0 << " |\n" << "| " << d0 - d1 << " |\n";
    TTC = ttc(1.0 / frameRate, d0, d1);
}

void associateKeyPointsToBoxes(const DataFrame &frame, map<KeyPointIdx, BoxIdx> &keyPointToBoxIdMap) {

    // loop over all key points
    for (int kptIdx = 0; kptIdx < frame.keypoints.size(); ++kptIdx) {
        auto kpt = frame.keypoints[kptIdx];
        vector<BoxIdx> enclosingBoxes;

        // loop over all boxes in frame, and find the boxes that contain the key point.
        for (auto &bbox: frame.boundingBoxes) {
            if (bbox.roi.contains(kpt.pt)) {
                enclosingBoxes.push_back(bbox.boxID);
            }
        }

        // key points contained in multiple boxes are ignored.
        if (enclosingBoxes.size() == 1) {
            keyPointToBoxIdMap[kptIdx] = enclosingBoxes[0];
        }
    }
}

void matchBoundingBoxes(const std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches,
                        const DataFrame &prevFrame,
                        const DataFrame &currFrame) {

    // associate key-points to bounding boxes, in both frames, and build maps associating key points to boxes.
    map<KeyPointIdx, BoxIdx> prevKeyPointToBoxIdMap, currKeyPointToBoxIdMap;

    associateKeyPointsToBoxes(prevFrame, prevKeyPointToBoxIdMap);
    associateKeyPointsToBoxes(currFrame, currKeyPointToBoxIdMap);

    // fill the multimap from Current Image ROI -> Previous Image ROI
    multimap<BoxIdx, BoxIdx> roiMatches;
    for (auto &match: currFrame.kptMatches) {

        // the query image is the previous frame.
        if (prevKeyPointToBoxIdMap.count(match.queryIdx) && currKeyPointToBoxIdMap.count(match.trainIdx)) {
            BoxIdx prevBoxIdx = prevKeyPointToBoxIdMap[match.queryIdx];
            BoxIdx currBoxIdx = currKeyPointToBoxIdMap[match.trainIdx];
            roiMatches.insert(BoxIdPair(prevBoxIdx, currBoxIdx));
        }
    }

    // loop over ROI bounding boxes in the previous image.
    for (auto &prevBB: prevFrame.boundingBoxes) {
        auto matchedBoxIndices = roiMatches.equal_range(prevBB.boxID);

        // when the prevBB has NO matches skip it.
        if (matchedBoxIndices.first == matchedBoxIndices.second) continue;

        // a map from matched ROI BoxIdx in the current image to number of occurrences in roiMatches
        map<BoxIdx, int> matchedBoxIdxToCountMap;

        // iterate over the ROI matches in the current image, and count them.
        for (auto it = matchedBoxIndices.first; it != matchedBoxIndices.second; ++it) {

            // Find occurrences of matchedBoxIdx
            // * if matchedBoxIdx isn't found then this is the first occurrence, otherwise iterate the count by 1.
            int matchedBoxIdx = it->second;
            auto matchedBoxIdxCountPair = matchedBoxIdxToCountMap.find(matchedBoxIdx);

            if (matchedBoxIdxCountPair == matchedBoxIdxToCountMap.end()) {
                matchedBoxIdxToCountMap.insert(BoxIdPair(matchedBoxIdx, 1));
            } else {
                matchedBoxIdxCountPair->second = 1 + matchedBoxIdxCountPair->second;
            }
        }

        // The matched boxIdx with the highest count is the best ROI match in the current image
        auto matchIdx =
                max_element(matchedBoxIdxToCountMap.begin(), matchedBoxIdxToCountMap.end(),
                            [](const BoxIdPair &p, const BoxIdPair &q) {
                                return p.second < q.second;
                            })->first;

        bbBestMatches.insert(BoxIdPair(prevBB.boxID, matchIdx));
    }
}

