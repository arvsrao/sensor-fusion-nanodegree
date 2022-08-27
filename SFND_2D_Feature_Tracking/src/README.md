## MP.1 Data Buffer Optimization

In lines 98 - 100 of *MidTermProject_Camera_Student.cpp* a ring buffer is implemented. First a new frame is appended to the end or back of the buffer (line 98)

    dataBuffer.push_back(frame); // [frame_0, frame_1, new_frame]

Then in line 101 the first element of the buffer is removed if the buffer size is greater than the maximum size.

    if (dataBuffer.size() > dataBufferSize) dataBuffer.erase(dataBuffer.begin());

## MP.2 Keypoint Detection

In lines 133 - 141 of *MidTermProject_Camera_Student.cpp* I completed the conditional statements, allowing selection of HARRIS, SIFT, and binary detectors. 
I declared function 

    void detKeypointsSIFT(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis=false);

on line 24 of *matching2D.cpp*, and then implemented it using the OpenCV api in *matching2D_Student.cpp*, lines 117 - 132. 
Then I implemented functions `detKeypointsModern` and `detKeypointsHarris` in **matching2D_Student.cpp** using the OpenCV api.
Furthermore, I implemented HARRIS by essentially copying my non-maximum suppression solution from an earlier 
exercise.

## MP.3 Keypoint Removal

I filtered-in key points contained in the suggested window by declaring an OpenCV `Rect`on line 152 of *MidTermProject_Camera_Student.cpp*
of the suggested size,

                    cv::Rect vehicleRect(535, 180, 180, 150);

The code in lines 153 - 162 checks if each detected key point is contained in the `Rect` declared in line 152, and above.
Those key points which are contained in the `Rect` are appended to a temporary collection, which is subsequently copied to 
the `keypoints` vector.

## MP.4 Keypoint Descriptors

BRIEF, ORB, FREAK, AKAZE and SIFT are implemented using the OpenCV api and made selectable in function `descKeypoints` 
of *matching2D_Student.cpp*.

## MP.5 Descriptor Matching

Inside function `matchDescriptors` I used the OpenCV api to implement FLANN matching, see line 22 of *matching2D_Student.cpp*. Additionally, I 
conditionally set the norm / distance in the BFMatcher depending on whether the descriptor is BINARY or HOG -- see line 17.

## MP.6 Descriptor Distance Ratio

In an `else if` block on lines 31 - 42 I added *k-NN* matching, using the OpenCV api. Following the call to the 
kNN matcher on line 34, kNN matches are filtered based on the descriptor distance ratio test. Only those first matches
which are not too close in distance -- as determined by the set ratio (here 0.8) -- to their associated second match are kept.

## Task MP.7

### Number of keypoint detected on the preceding vehicle

| Frame | SHITOMASI | HARRIS | SIFT | FAST | BRISK | ORB | AKAZE | 
|-------|-----------|--------|------|------|-------|-----|-------|
| 0 | 125 | 169 | 138 | 419 | 264 | 92  | 166 | 
| 1 | 118 | 154 | 132 | 427 | 282 | 102 | 157 | 
| 2 | 123 | 179 | 124 | 404 | 282 | 106 | 161 | 
| 3 | 120 | 173 | 137 | 423 | 277 | 113 | 155 | 
| 4 | 120 | 193 | 134 | 386 | 297 | 109 | 163 | 
| 5 | 113 | 347 | 140 | 414 | 279 | 125 | 164 | 
| 6 | 114 | 151 | 137 | 418 | 289 | 130 | 173 | 
| 7 | 123 | 239 | 148 | 406 | 272 | 129 | 175 | 
| 8 | 111 | 195 | 159 | 396 | 266 | 127 | 177 | 
| 9 | 112 | 262 | 137 | 401 | 254 | 128 | 179 |

### Detected keypoint mean diameters on preceding vehicle

| Frame | SHITOMASI | HARRIS | SIFT | FAST | BRISK | ORB | AKAZE | 
|-------|-----------|--------|------|------|-------|-----|-------|
| 0 | 4 | 4.52071 | 4.98472 | 7 | 21.5492 | 57.0723 | 7.72918 | 
| 1 | 4 | 4.46753 | 5.08981 | 7 | 21.7853 | 57.2273 | 7.49021 | 
| 2 | 4 | 4.6257  | 4.93926 | 7 | 21.6509 | 56.4948 | 7.45212 | 
| 3 | 4 | 4.61272 | 4.73123 | 7 | 20.3583 | 55.1436 | 7.57523 | 
| 4 | 4 | 4.75648 | 4.7196  | 7 | 22.5911 | 56.7442 | 7.73319 | 
| 5 | 4 | 5.3487  | 4.68398 | 7 | 22.9442 | 56.6367 | 7.68804 | 
| 6 | 4 | 4.49007 | 5.40798 | 7 | 21.8014 | 56.7683 | 7.73879 | 
| 7 | 4 | 4.97071 | 4.62187 | 7 | 22.1472 | 55.4296 | 7.82613 | 
| 8 | 4 | 4.86154 | 5.51997 | 7 | 22.5558 | 54.6723 | 7.81556 | 
| 9 | 4 | 5.14504 | 5.62509 | 7 | 22.0389 | 54.3885 | 7.88576 | 

| Detector | Neighborhood Shape Description|
|-------|-----------|
| SHITOMASI | keypoints capture license plate, top & side of car rear, and a car further up in the right lane. | 
| HARRIS | keypoints capture rear tail lights, parts of the rear window, and edges of the roof. The rears of two cars in the right lane are also captured. |
| SIFT | keypoints capture rear tail lights, parts of the rear window, but also parts of the road bed around the online of the car rear's shadow and a white lane marker. The rears of two cars in the right lane are also captured. |
| FAST | keypoints capture license plate, top & side of car rear, portions of the outline of car rear's shadow, and and a car further up in the right lane. |
| BRISK | keypoint diameters are generally large and cover the license plate, sides of the car rear and car roof, as well as part of the right rear tire. |
| ORB | keypoint diameters are generally large and cover the rear lights and roof, but also features of vehicles further up in left and right lanes. |
| AKAZE | keypoints capture rear tail lights, sides of the rear, and top of the rear window, parts of the roof, and back right tire, and portions of the outline of car rear's shadow. Also features of vehicles further up in left and right lanes. |

## Task MP 8. & MP.9

### Matched Key Points for all Possible Detector-Descriptor Combinations

| Frame | Detector | Descriptor | # of Matches | Time |
|-------|----------|------------|--------------|------|
| 1 | SIFT | SIFT | 82 | 204.292 ms |
| 2 | SIFT | SIFT | 81 | 192.745 ms |
| 3 | SIFT | SIFT | 85 | 185.109 ms |
| 4 | SIFT | SIFT | 93 | 198.851 ms |
| 5 | SIFT | SIFT | 90 | 214.982 ms |
| 6 | SIFT | SIFT | 81 | 191.227 ms |
| 7 | SIFT | SIFT | 82 | 191.115 ms |
| 8 | SIFT | SIFT | 102 | 195.966 ms |
| 9 | SIFT | SIFT | 104 | 191.724 ms |
| 1 | AKAZE | AKAZE | 138 | 180.085 ms |
| 2 | AKAZE | AKAZE | 138 | 194.593 ms |
| 3 | AKAZE | AKAZE | 133 | 184.543 ms |
| 4 | AKAZE | AKAZE | 127 | 184.317 ms |
| 5 | AKAZE | AKAZE | 129 | 187.465 ms |
| 6 | AKAZE | AKAZE | 146 | 184.094 ms |
| 7 | AKAZE | AKAZE | 147 | 183.782 ms |
| 8 | AKAZE | AKAZE | 151 | 184.839 ms |
| 9 | AKAZE | AKAZE | 150 | 174.466 ms |
| 1 | SHITOMASI | BRISK | 84 | 30.2215 ms |
| 2 | SHITOMASI | BRISK | 80 | 35.6497 ms |
| 3 | SHITOMASI | BRISK | 73 | 38.6561 ms |
| 4 | SHITOMASI | BRISK | 77 | 40.7216 ms |
| 5 | SHITOMASI | BRISK | 74 | 47.5392 ms |
| 6 | SHITOMASI | BRISK | 70 | 43.8257 ms |
| 7 | SHITOMASI | BRISK | 79 | 34.9504 ms |
| 8 | SHITOMASI | BRISK | 81 | 45.3016 ms |
| 9 | SHITOMASI | BRISK | 72 | 52.4022 ms |
| 1 | HARRIS | BRISK | 18 | 41.9293 ms |
| 2 | HARRIS | BRISK | 15 | 53.7804 ms |
| 3 | HARRIS | BRISK | 25 | 48.3543 ms |
| 4 | HARRIS | BRISK | 21 | 36.6832 ms |
| 5 | HARRIS | BRISK | 23 | 43.2511 ms |
| 6 | HARRIS | BRISK | 46 | 56.0122 ms |
| 7 | HARRIS | BRISK | 17 | 56.2558 ms |
| 8 | HARRIS | BRISK | 45 | 46.5579 ms |
| 9 | HARRIS | BRISK | 37 | 56.9139 ms |
| 1 | FAST | BRISK | 256 | 38.5994 ms |
| 2 | FAST | BRISK | 243 | 38.662 ms |
| 3 | FAST | BRISK | 241 | 49.2731 ms |
| 4 | FAST | BRISK | 239 | 41.9493 ms |
| 5 | FAST | BRISK | 215 | 32.9091 ms |
| 6 | FAST | BRISK | 251 | 35.0214 ms |
| 7 | FAST | BRISK | 248 | 39.6527 ms |
| 8 | FAST | BRISK | 243 | 47.7433 ms |
| 9 | FAST | BRISK | 247 | 35.8013 ms |
| 1 | BRISK | BRISK | 171 | 106.003 ms |
| 2 | BRISK | BRISK | 176 | 101.714 ms |
| 3 | BRISK | BRISK | 157 | 117.7 ms |
| 4 | BRISK | BRISK | 176 | 127.774 ms |
| 5 | BRISK | BRISK | 174 | 105.689 ms |
| 6 | BRISK | BRISK | 188 | 121.499 ms |
| 7 | BRISK | BRISK | 173 | 99.5329 ms |
| 8 | BRISK | BRISK | 171 | 105.7 ms |
| 9 | BRISK | BRISK | 184 | 115.249 ms |
| 1 | ORB | BRISK | 73 | 39.6151 ms |
| 2 | ORB | BRISK | 74 | 36.1322 ms |
| 3 | ORB | BRISK | 79 | 33.5897 ms |
| 4 | ORB | BRISK | 85 | 41.0627 ms |
| 5 | ORB | BRISK | 79 | 52.4044 ms |
| 6 | ORB | BRISK | 92 | 44.9995 ms |
| 7 | ORB | BRISK | 90 | 37.6882 ms |
| 8 | ORB | BRISK | 88 | 34.1825 ms |
| 9 | ORB | BRISK | 91 | 43.8841 ms |
| 1 | SHITOMASI | BRIEF | 96 | 16.8081 ms |
| 2 | SHITOMASI | BRIEF | 93 | 14.2291 ms |
| 3 | SHITOMASI | BRIEF | 92 | 11.7291 ms |
| 4 | SHITOMASI | BRIEF | 89 | 13.6664 ms |
| 5 | SHITOMASI | BRIEF | 92 | 14.6581 ms |
| 6 | SHITOMASI | BRIEF | 93 | 15.3099 ms |
| 7 | SHITOMASI | BRIEF | 85 | 17.9913 ms |
| 8 | SHITOMASI | BRIEF | 91 | 13.8675 ms |
| 9 | SHITOMASI | BRIEF | 85 | 13.633 ms |
| 1 | HARRIS | BRIEF | 21 | 13.4567 ms |
| 2 | HARRIS | BRIEF | 16 | 11.8906 ms |
| 3 | HARRIS | BRIEF | 25 | 10.4148 ms |
| 4 | HARRIS | BRIEF | 23 | 15.2773 ms |
| 5 | HARRIS | BRIEF | 20 | 21.4451 ms |
| 6 | HARRIS | BRIEF | 47 | 21.065 ms |
| 7 | HARRIS | BRIEF | 15 | 17.7168 ms |
| 8 | HARRIS | BRIEF | 43 | 13.9126 ms |
| 9 | HARRIS | BRIEF | 47 | 11.3208 ms |
| 1 | FAST | BRIEF | 320 | 6.37256 ms |
| 2 | FAST | BRIEF | 332 | 5.37765 ms |
| 3 | FAST | BRIEF | 299 | 6.35343 ms |
| 4 | FAST | BRIEF | 331 | 8.47828 ms |
| 5 | FAST | BRIEF | 276 | 8.26329 ms |
| 6 | FAST | BRIEF | 327 | 7.27918 ms |
| 7 | FAST | BRIEF | 324 | 6.35359 ms |
| 8 | FAST | BRIEF | 315 | 6.40339 ms |
| 9 | FAST | BRIEF | 307 | 5.7633 ms |
| 1 | BRISK | BRIEF | 178 | 98.9941 ms |
| 2 | BRISK | BRIEF | 205 | 77.3526 ms |
| 3 | BRISK | BRIEF | 185 | 100.031 ms |
| 4 | BRISK | BRIEF | 179 | 81.2121 ms |
| 5 | BRISK | BRIEF | 183 | 93.7975 ms |
| 6 | BRISK | BRIEF | 195 | 87.3218 ms |
| 7 | BRISK | BRIEF | 207 | 76.8714 ms |
| 8 | BRISK | BRIEF | 189 | 100.922 ms |
| 9 | BRISK | BRIEF | 183 | 80.7111 ms |
| 1 | ORB | BRIEF | 49 | 7.96819 ms |
| 2 | ORB | BRIEF | 43 | 8.81446 ms |
| 3 | ORB | BRIEF | 45 | 8.03785 ms |
| 4 | ORB | BRIEF | 59 | 8.86402 ms |
| 5 | ORB | BRIEF | 53 | 7.80518 ms |
| 6 | ORB | BRIEF | 78 | 9.40432 ms |
| 7 | ORB | BRIEF | 68 | 9.18079 ms |
| 8 | ORB | BRIEF | 84 | 11.6039 ms |
| 9 | ORB | BRIEF | 66 | 14.4545 ms |
| 1 | SHITOMASI | ORB | 87 | 13.0505 ms |
| 2 | SHITOMASI | ORB | 82 | 12.8592 ms |
| 3 | SHITOMASI | ORB | 87 | 11.4871 ms |
| 4 | SHITOMASI | ORB | 90 | 11.7 ms |
| 5 | SHITOMASI | ORB | 83 | 14.2393 ms |
| 6 | SHITOMASI | ORB | 77 | 14.9042 ms |
| 7 | SHITOMASI | ORB | 84 | 16.8919 ms |
| 8 | SHITOMASI | ORB | 86 | 16.6876 ms |
| 9 | SHITOMASI | ORB | 89 | 13.0863 ms |
| 1 | HARRIS | ORB | 15 | 11.6547 ms |
| 2 | HARRIS | ORB | 16 | 12.068 ms |
| 3 | HARRIS | ORB | 16 | 11.3727 ms |
| 4 | HARRIS | ORB | 20 | 15.3076 ms |
| 5 | HARRIS | ORB | 12 | 17.2498 ms |
| 6 | HARRIS | ORB | 36 | 17.2359 ms |
| 7 | HARRIS | ORB | 16 | 17.7756 ms |
| 8 | HARRIS | ORB | 39 | 21.6249 ms |
| 9 | HARRIS | ORB | 30 | 16.6023 ms |
| 1 | FAST | ORB | 306 | 9.83973 ms |
| 2 | FAST | ORB | 314 | 10.2529 ms |
| 3 | FAST | ORB | 295 | 8.26544 ms |
| 4 | FAST | ORB | 318 | 9.638 ms |
| 5 | FAST | ORB | 284 | 8.50368 ms |
| 6 | FAST | ORB | 312 | 7.21932 ms |
| 7 | FAST | ORB | 323 | 5.97713 ms |
| 8 | FAST | ORB | 306 | 5.89872 ms |
| 9 | FAST | ORB | 304 | 5.50271 ms |
| 1 | BRISK | ORB | 160 | 87.1796 ms |
| 2 | BRISK | ORB | 171 | 93.7476 ms |
| 3 | BRISK | ORB | 157 | 96.0365 ms |
| 4 | BRISK | ORB | 170 | 85.1178 ms |
| 5 | BRISK | ORB | 154 | 99.0492 ms |
| 6 | BRISK | ORB | 180 | 96.7037 ms |
| 7 | BRISK | ORB | 171 | 95.0422 ms |
| 8 | BRISK | ORB | 175 | 77.1595 ms |
| 9 | BRISK | ORB | 172 | 96.0882 ms |
| 1 | ORB | ORB | 65 | 19.0878 ms |
| 2 | ORB | ORB | 69 | 16.1475 ms |
| 3 | ORB | ORB | 71 | 16.4063 ms |
| 4 | ORB | ORB | 85 | 18.2773 ms |
| 5 | ORB | ORB | 91 | 21.3348 ms |
| 6 | ORB | ORB | 101 | 23.4495 ms |
| 7 | ORB | ORB | 95 | 23.6178 ms |
| 8 | ORB | ORB | 93 | 17.9829 ms |
| 9 | ORB | ORB | 91 | 18.6853 ms |
| 1 | SHITOMASI | FREAK | 68 | 44.1487 ms |
| 2 | SHITOMASI | FREAK | 69 | 34.9394 ms |
| 3 | SHITOMASI | FREAK | 64 | 31.6048 ms |
| 4 | SHITOMASI | FREAK | 64 | 37.9966 ms |
| 5 | SHITOMASI | FREAK | 63 | 51.5015 ms |
| 6 | SHITOMASI | FREAK | 61 | 40.7816 ms |
| 7 | SHITOMASI | FREAK | 62 | 31.8165 ms |
| 8 | SHITOMASI | FREAK | 62 | 42.8343 ms |
| 9 | SHITOMASI | FREAK | 62 | 45.715 ms |
| 1 | HARRIS | FREAK | 21 | 34.9125 ms |
| 2 | HARRIS | FREAK | 19 | 36.3903 ms |
| 3 | HARRIS | FREAK | 18 | 47.5937 ms |
| 4 | HARRIS | FREAK | 22 | 41.4278 ms |
| 5 | HARRIS | FREAK | 19 | 33.5119 ms |
| 6 | HARRIS | FREAK | 44 | 30.7088 ms |
| 7 | HARRIS | FREAK | 14 | 41.8385 ms |
| 8 | HARRIS | FREAK | 45 | 51.1878 ms |
| 9 | HARRIS | FREAK | 43 | 48.3013 ms |
| 1 | FAST | FREAK | 251 | 42.1917 ms |
| 2 | FAST | FREAK | 250 | 43.4218 ms |
| 3 | FAST | FREAK | 228 | 38.6382 ms |
| 4 | FAST | FREAK | 252 | 27.1233 ms |
| 5 | FAST | FREAK | 234 | 26.7089 ms |
| 6 | FAST | FREAK | 269 | 36.8734 ms |
| 7 | FAST | FREAK | 252 | 40.8223 ms |
| 8 | FAST | FREAK | 243 | 28.5579 ms |
| 9 | FAST | FREAK | 246 | 27.5736 ms |
| 1 | BRISK | FREAK | 160 | 101.291 ms |
| 2 | BRISK | FREAK | 178 | 119.675 ms |
| 3 | BRISK | FREAK | 156 | 138.095 ms |
| 4 | BRISK | FREAK | 173 | 124.946 ms |
| 5 | BRISK | FREAK | 160 | 106.024 ms |
| 6 | BRISK | FREAK | 183 | 119.005 ms |
| 7 | BRISK | FREAK | 169 | 116.844 ms |
| 8 | BRISK | FREAK | 179 | 101.78 ms |
| 9 | BRISK | FREAK | 168 | 116.8 ms |
| 1 | ORB | FREAK | 42 | 34.491 ms |
| 2 | ORB | FREAK | 36 | 47.8722 ms |
| 3 | ORB | FREAK | 45 | 43.5392 ms |
| 4 | ORB | FREAK | 47 | 38.3943 ms |
| 5 | ORB | FREAK | 44 | 32.9999 ms |
| 6 | ORB | FREAK | 51 | 37.2083 ms |
| 7 | ORB | FREAK | 52 | 44.8422 ms |
| 8 | ORB | FREAK | 49 | 32.2681 ms |
| 9 | ORB | FREAK | 55 | 28.0766 ms |

### Detector-Descriptor Recommendation

I used the following criteria to compare detector-descriptor pairs:
* The detector should not detect too many key points, because many will be redundant or noisy--not located on car being followed. Subsequently, the matcher will waste time matching bad key points. 
* The key points should not have too large diameters, otherwise descriptor will be computed from large areas including the roadway or nearby vehicles, features of the scene which may not be stable.
* The combined detection - description loop should be fast, under 50 seconds, say.

| Rank | Detector  | Descriptor | Average # of Matches |  Average Time |  Reason for Ranking |
|------|-----------|------------|----------------------|---------------|---------------------|
| 1    | HARRIS    | BRIEF      |   28.5556            | 10.2293 ms    | HARRIS key points have relatively small diameters, which is likely the reason why there are a low number of matches. Many of the key points don't match. Most of the matches are on the rear of the car. The HARRIS - BRIEF pair is among the fastest. |
| 2    | HARRIS    | ORB        |   22.2222            | 10.3836 ms    | The HARRIS - ORB pair matches slightly fewer than HARRIS - BRIEF. However, the matched key points are high quality and well matched. Though there are some noisy matches. |
| 3    | SHITOMASI | ORB        |      85              | 9.49596 ms    | This pairing is among the fastest. SHITOMASI key points have relatively small diameters, like HARRIS. However SHITOMASI - ORB is able to match more of the detected key points. A large number cover the rear window. Though some of the matches are erroneous -- a cluster of key points in the left lane are matched to key points in the right lane. |
