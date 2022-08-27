/* INCLUDES FOR THIS PROJECT */
#define MP7 false
#define NUM_KEYPOINTS false
#define MP8 true

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <set>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

struct Pair {
    string detectorType, descriptorType;
};

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */

   // string detectorType = "HARRIS";
   // string descriptorType = "ORB";
    set<string> hogDetectors = { "SHITOMASI", "HARRIS", "SIFT" };
    vector<string> detectorTypes = { "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB" };
    vector<string> descriptorTypes = { "BRISK", "BRIEF", "ORB", "FREAK" };

    vector<Pair> pairs = {
            Pair { "SIFT", "SIFT"},
            Pair { "AKAZE", "AKAZE"}
    };

    for (const auto& descriptorType : descriptorTypes) {
        for (const auto &detectorType: detectorTypes) pairs.push_back(Pair { detectorType, descriptorType });
    }

    if (MP8) {
            std::cout << "| Frame | Detector | Descriptor | # of Matches | Time |" << "\n";
            std::cout << "|-------|----------|------------|--------------|------|" << "\n";
    }

    vector<Pair> mp8 = { Pair { "HARRIS", "ORB" } };

   for (const auto& pair : mp8 ) {
        dataBuffer.clear();

        for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++) {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;


        // Implement Ring Buffer. Remove the 'oldFrame' element from buffer. Should be like a queue.
        // TODO: For ring buffers, you typically remove elements before inserting. However, because
        //  this is not a true ring buffer, this is acceptable but keep this in mind for the future.
        if (dataBuffer.size() > dataBufferSize) dataBuffer.erase(dataBuffer.begin());
        dataBuffer.push_back(frame); // [frame_0, frame_1, new_frame]

        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        if (MP7) {
            string column_heads = "| ";
            string pipes_and_dashes;
            for (auto& detector : detectorTypes) { column_heads += detector + " | ";}
            for (auto& detector : detectorTypes) {
                pipes_and_dashes.append("-");
                for (int i = 0; i < detector.size(); i++) pipes_and_dashes.append("-");
                pipes_and_dashes += "-|";
            }

            if (imgIndex == 0) {
                std::cout << "| Frame " << column_heads << "\n";
                std::cout << "|-------|" << pipes_and_dashes << "\n";
            }
            std::cout << "| " << imgIndex << " | ";
        }

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
        double t = (double) cv::getTickCount();

                if (pair.detectorType.compare("SHITOMASI") == 0) {
                    detKeypointsShiTomasi(keypoints, imgGray, false);
                } else if (pair.detectorType.compare("HARRIS") == 0) {
                    detKeypointsHarris(keypoints, imgGray, false);
                } else if (pair.detectorType.compare("SIFT") == 0) {
                    detKeypointsSIFT(keypoints, imgGray, false);
                } else {
                    detKeypointsModern(keypoints, imgGray, pair.detectorType, false);
                }

                //cout << detectorType << " detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

                //// EOF STUDENT ASSIGNMENT

                //// STUDENT ASSIGNMENT
                //// TASK MP.3 -> only keep keypoints on the preceding vehicle

                // only keep keypoints on the preceding vehicle
                bool bFocusOnVehicle = true;
                bool MY_WAY = false;
                cv::Rect vehicleRect(535, 180, 180, 150);
                if (bFocusOnVehicle) {

                    // reviewer suggested way to filter
                    keypoints.erase(std::remove_if(keypoints.begin(),
                                                   keypoints.end(),
                                                   [&vehicleRect](const cv::KeyPoint &kpt) -> bool {
                                                       return !vehicleRect.contains(kpt.pt);
                                                   }),
                                    keypoints.end());

                    if (MY_WAY) {
                        vector<cv::KeyPoint> temp;

                        // remove key points outside leading vehicle bounding box (C++11 and later)
                        for (auto &it: keypoints) {
                            if (vehicleRect.contains(it.pt)) temp.push_back(it);
                        }
                        keypoints.clear();
                        keypoints = move(temp);
                    }
                }
                float min_x = std::min_element(keypoints.begin(), keypoints.end(),
                                               [](const cv::KeyPoint &a, const cv::KeyPoint &b) {
                                                   return a.pt.x < b.pt.x;
                                               })->pt.x;
                float max_x = std::max_element(keypoints.begin(), keypoints.end(),
                                               [](const cv::KeyPoint &a, const cv::KeyPoint &b) {
                                                   return a.pt.x < b.pt.x;
                                               })->pt.x;
                float min_y = std::min_element(keypoints.begin(), keypoints.end(),
                                               [](const cv::KeyPoint &a, const cv::KeyPoint &b) {
                                                   return a.pt.y < b.pt.y;
                                               })->pt.y;
                float max_y = std::max_element(keypoints.begin(), keypoints.end(),
                                               [](const cv::KeyPoint &a, const cv::KeyPoint &b) {
                                                   return a.pt.y < b.pt.y;
                                               })->pt.y;
                cv::Rect2f keypoint_neighborhood(min_x, min_y, max_x - min_x, max_y - min_y);
#

                if (MP7 && !NUM_KEYPOINTS) {
                    float keypoints_diameter = 0.0;
                    for (auto &kp: keypoints) keypoints_diameter += kp.size;
                    std::cout << keypoints_diameter / keypoints.size() << " | ";
                } else if (MP7 && NUM_KEYPOINTS) {
                    std::cout << keypoints.size() << " | ";
                }

                if (MP7) std::cout << "" << std::endl;
                //// EOF STUDENT ASSIGNMENT

                // optional : limit number of keypoints (helpful for debugging and learning)
                bool bLimitKpts = false;
                if (bLimitKpts) {
                    int maxKeypoints = 50;

                    if (pair.detectorType.compare("SHITOMASI") ==
                        0) { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                        keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                    }
                    cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                    cout << " NOTE: Keypoints have been limited!" << endl;
                }

                // push keypoints and descriptor for current frame to end of data buffer
                (dataBuffer.end() - 1)->keypoints = keypoints;
                cout << "#2 : DETECT KEYPOINTS done" << endl;

                /* EXTRACT KEYPOINT DESCRIPTORS */

                //// STUDENT ASSIGNMENT
                //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
                //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

                //t = (double) cv::getTickCount();

                cv::Mat descriptors;

                // string descriptorType = "BRISK"; // BRIEF, ORB, FREAK, AKAZE, SIFT
                descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors,
                              pair.descriptorType);

                //// EOF STUDENT ASSIGNMENT

                // push descriptors for current frame to end of data buffer
                (dataBuffer.end() - 1)->descriptors = descriptors;

               // std::cout << pair.descriptorType << " descriptor with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

                cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

                if (dataBuffer.size() > 1) // wait until at least two images have been processed
                {
                    /* MATCH KEYPOINT DESCRIPTORS */

                    vector<cv::DMatch> matches;
                    string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
                    string descriptor_type = hogDetectors.count(pair.detectorType) ? "DES_HOG"
                                                                             : "DES_BINARY"; // DES_BINARY, DES_HOG
                    string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

                    //// STUDENT ASSIGNMENT
                    //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                    //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

                    matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                     (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                     matches, descriptor_type, matcherType, selectorType);

                    t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();

                    if (MP8) {
                        std::cout << "| " << imgIndex << " | " << pair.detectorType << " | " << pair.descriptorType << " | " << matches.size() << " | "
                                  << 1000 * t / 1.0 << " ms |" << std::endl;
                    }

                    //// EOF STUDENT ASSIGNMENT

                    // store matches in current data frame
                    (dataBuffer.end() - 1)->kptMatches = matches;

                    cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

                    // visualize matches between current and previous image
                    bVis = true;
                    if (bVis) {
                        cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                        cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                        (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                        matches, matchImg,
                                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                                        vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                        string windowName = "Matching keypoints between two camera images";
                        cv::namedWindow(windowName, 7);
                        cv::imshow(windowName, matchImg);
                        cout << "Press key to continue to next image" << endl;
                        cv::waitKey(0); // wait for key to be pressed
                    }
                    bVis = false;
                }
            } // eof loop over all images
    } // loop over pairs

    return 0;
}