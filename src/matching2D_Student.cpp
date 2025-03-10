// #include <numeric>
// #include "matching2D.hpp"

// using namespace std;

// // Find best matches for keypoints in two camera images based on several matching methods
// void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
//                       std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
// {
//     // configure matcher
    
//     bool crossCheck = true;
//     cv::Ptr<cv::DescriptorMatcher> matcher;

//     if (matcherType.compare("MAT_BF") == 0)
//     {
//         int normType = descriptorType.compare("DES_BINARY")==0 ? cv::NORM_HAMMING : cv::NORM_L2;
// //       	std::cout<<normType<<" norm used "<<std::endl;
//         matcher = cv::BFMatcher::create(normType, crossCheck);
//         if (descSource.type()!=CV_32S || descSource.type()!=CV_8U)
//         {
//             descSource.convertTo(descSource,CV_8U);
//             descRef.convertTo(descRef,CV_8U);
//         }
//     }
//     else if (matcherType.compare("MAT_FLANN") == 0)
//     {
//         if (descSource.type()!=CV_32F)
//         {
//         	descSource.convertTo(descSource,CV_32F);
//         	descRef.convertTo(descRef,CV_32F);
//         }

//         matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
//     }

//     // perform matching task
//     if (selectorType.compare("SEL_NN") == 0)
//     { // nearest neighbor (best match)

//         matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
//     }
//     else if (selectorType.compare("SEL_KNN") == 0)
//     { // k nearest neighbors (k=2)
//     	std::vector<std::vector<cv::DMatch>> knnMatches;
//     	matcher->knnMatch(descSource,descRef,knnMatches,2);

//     	float minDistanceRatio = 0.8;
//     	for (auto it=knnMatches.begin(); it!=knnMatches.end(); ++it)
//     	{
//     		if ((*it)[0].distance < minDistanceRatio*(*it)[1].distance)
//     			matches.push_back((*it)[0]);
//     	}
//       	knnMatches.erase(knnMatches.begin(),knnMatches.end());
//     }
// }

// // Use one of several types of state-of-art descriptors to uniquely identify keypoints
// void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
// {
//     // select appropriate descriptor
// 	//// -> BRIEF, ORB, FREAK, AKAZE, SIFT

//     cv::Ptr<cv::DescriptorExtractor> extractor;
//     if (descriptorType.compare("BRISK") == 0)
//     {
//         int threshold = 30;        // FAST/AGAST detection threshold score.
//         int octaves = 3;           // detection octaves (use 0 to do single scale)
//         float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

//         extractor = cv::BRISK::create(threshold, octaves, patternScale);
//     }    
//     else if (descriptorType.compare("BRIEF")==0)
//     	extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
//     else if (descriptorType.compare("ORB")==0)
//     	extractor = cv::ORB::create(10);
//     else if (descriptorType.compare("FREAK")==0)
//     	extractor = cv::xfeatures2d::FREAK::create();    
//     else if (descriptorType.compare("AKAZE")==0)
//     	extractor = cv::AKAZE::create();
//     else if (descriptorType.compare("SIFT")==0)
//     	extractor = cv::xfeatures2d::SIFT::create();

//     // perform feature description
//     double t = (double)cv::getTickCount();
//     extractor->compute(img, keypoints, descriptors);
//     t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
// //     cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
  	
// }

// // Detect keypoints in image using the traditional Shi-Thomasi detector
// void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
// {
//     // compute detector parameters based on image size
//     int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
//     double maxOverlap = 0.0; // max. permissible overlap between two features in %
//     double minDistance = (1.0 - maxOverlap) * blockSize;
//     int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

//     double qualityLevel = 0.01; // minimal accepted quality of image corners
//     double k = 0.04;

//     // Apply corner detection
//     double t = (double)cv::getTickCount();
//     vector<cv::Point2f> corners;
//     cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

//     // add corners to result vector
//     for (auto it = corners.begin(); it != corners.end(); ++it)
//     {

//         cv::KeyPoint newKeyPoint;
//         newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
//         newKeyPoint.size = blockSize;
//         keypoints.push_back(newKeyPoint);
//     }
//     t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
// //     cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

//     // visualize results
//     if (bVis)
//     {
//         cv::Mat visImage = img.clone();
//         cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//         string windowName = "Shi-Tomasi Corner Detector Results";
//         cv::namedWindow(windowName, 6);
//         imshow(windowName, visImage);
//         cv::waitKey(0);
//     }
  	
// }

// void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
// {
  
// 	cv::Mat dst = img.clone();
// 	cv::Mat dst_norm, dst_norm_scaled;
  	
//   	int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
//     int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
//     int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
//     double k = 0.04;       // Harris parameter (see equation for details)
// 	double t = (double) cv::getTickCount();
//     // Detect Harris corners and normalize output
// //     cv::Mat dst, dst_norm, dst_norm_scaled;
//     dst = cv::Mat::zeros(img.size(), CV_32FC1);
//     cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
//     cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
//     cv::convertScaleAbs(dst_norm, dst_norm_scaled);

//     // visualize results
// //     string windowName = "Harris Corner Detector Response Matrix";
// //     cv::namedWindow(windowName, 4);
// //     cv::imshow(windowName, dst_norm_scaled);
// //     cv::waitKey(0);

//     // STUDENTS NEET TO ENTER THIS CODE (C3.2 Atom 4)

//     // Look for prominent corners and instantiate keypoints
// //     vector<cv::KeyPoint> keypoints;
//     double maxOverlap = 0.0; // max. permissible overlap between two features in %, used during non-maxima suppression
//     for (size_t j = 0; j < dst_norm.rows; j++)
//     {
//         for (size_t i = 0; i < dst_norm.cols; i++)
//         {
//             int response = (int)dst_norm.at<float>(j, i);
//             if (response > minResponse)
//             { // only store points above a threshold

//                 cv::KeyPoint newKeyPoint;
//                 newKeyPoint.pt = cv::Point2f(i, j);
//                 newKeyPoint.size = 2 * apertureSize;
//                 newKeyPoint.response = response;

//                 // perform non-maximum suppression (NMS) in local neighbourhood around new key point
//                 bool bOverlap = false;
//                 for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
//                 {
//                     double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
//                     if (kptOverlap > maxOverlap)
//                     {
//                         bOverlap = true;
//                         if (newKeyPoint.response > (*it).response)
//                         {                      // if overlap is >t AND response is higher for new kpt
//                             *it = newKeyPoint; // replace old key point with new one
//                             break;             // quit loop over keypoints
//                         }
//                     }
//                 }
                
//             }
//         } // eof loop over cols
//     }     // eof loop over rows
  	

//     t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
// //     cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
// 	if (bVis)
// 	{
// 		string windowName = "Harris Keypoints";
// 		cv::Mat visImg = dst_norm_scaled.clone();
// 		cv::drawKeypoints(dst_norm_scaled,keypoints,visImg,cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
// 		cv::imshow(windowName,visImg);
// 	}

// }

// void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
// {
//     //// STUDENT ASSIGNMENT
//     //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
//     //// -> FAST, BRISK, ORB, AKAZE, SIFT
//     cv::Ptr<cv::FeatureDetector> detector;
//     string windowName;
//     double t = (double)cv::getTickCount();
// 	if (detectorType.compare("FAST"))
// 	{
// 		int threshold = 100;
// 		detector = cv::FastFeatureDetector::create(threshold);
// 		detector->detect(img,keypoints);
// 		windowName = "FAST features";

// 	}

// 	if (detectorType.compare("BRISK")==0)
// 	{
// 		detector = cv::BRISK::create();
// 		detector->detect(img,keypoints);
// 		windowName = "BRISK features";
// 	}

// 	if (detectorType.compare("ORB")==0)
// 	{
// 		int nfeatures = 100;
// 		detector = cv::ORB::create(nfeatures);
// 		detector->detect(img,keypoints);
// 		windowName = "ORB features";
// 	}

// 	if (detectorType.compare("AKAZE")==0)
// 	{
// 		detector = cv::AKAZE::create();
// 		detector->detect(img,keypoints);
// 		windowName = "AKAZE features";
// 	}

// 	if (detectorType.compare("SIFT")==0)
// 	{
// 		detector = cv::xfeatures2d::SIFT::create();
// 		detector->detect(img,keypoints);
// 		windowName = "SIFT features";
// 	}
//     t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
// //     cout << detectorType << " with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
// 	if (bVis)
// 	{
// 		cv::Mat visImg = img.clone();
// 		cv::drawKeypoints(img,keypoints,visImg,cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
// 		cv::imshow(windowName,visImg);
// 	}
  
// }

#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = cv::NORM_HAMMING;
        matcher = cv::BFMatcher::create(normType, crossCheck);
      	if (descSource.type()!=CV_32S || descSource.type()!=CV_8U)
        {
            descSource.convertTo(descSource,CV_8U);
            descRef.convertTo(descRef,CV_8U);
        }
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
     	if (descSource.type()!=CV_32F)
        {
        	descSource.convertTo(descSource,CV_32F);
        	descRef.convertTo(descRef,CV_32F);
        }

        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        std::vector<std::vector<cv::DMatch>> knnMatches;
    	matcher->knnMatch(descSource,descRef,knnMatches,2);

    	float minDistanceRatio = 0.8;
    	for (auto it=knnMatches.begin(); it!=knnMatches.end(); ++it)
    	{
    		if ((*it)[0].distance < minDistanceRatio*(*it)[1].distance)
    			matches.push_back((*it)[0]);
    	}
      	knnMatches.erase(knnMatches.begin(),knnMatches.end());
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF")==0)
    	extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    else if (descriptorType.compare("ORB")==0)
    	extractor = cv::ORB::create(10);
    else if (descriptorType.compare("FREAK")==0)
    	extractor = cv::xfeatures2d::FREAK::create();    
    else if (descriptorType.compare("AKAZE")==0)
    	extractor = cv::AKAZE::create();
    else if (descriptorType.compare("SIFT")==0)
    	extractor = cv::xfeatures2d::SIFT::create();

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
//     cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
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
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
//     cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

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

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
  
	cv::Mat dst = img.clone();
	cv::Mat dst_norm, dst_norm_scaled;
  	
  	int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)
	double t = (double) cv::getTickCount();
    // Detect Harris corners and normalize output
//     cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // visualize results
//     string windowName = "Harris Corner Detector Response Matrix";
//     cv::namedWindow(windowName, 4);
//     cv::imshow(windowName, dst_norm_scaled);
//     cv::waitKey(0);

    // STUDENTS NEET TO ENTER THIS CODE (C3.2 Atom 4)

    // Look for prominent corners and instantiate keypoints
//     vector<cv::KeyPoint> keypoints;
    double maxOverlap = 0.0; // max. permissible overlap between two features in %, used during non-maxima suppression
    for (size_t j = 0; j < dst_norm.rows; j++)
    {
        for (size_t i = 0; i < dst_norm.cols; i++)
        {
            int response = (int)dst_norm.at<float>(j, i);
            if (response > minResponse)
            { // only store points above a threshold

                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = response;

                // perform non-maximum suppression (NMS) in local neighbourhood around new key point
                bool bOverlap = false;
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                    if (kptOverlap > maxOverlap)
                    {
                        bOverlap = true;
                        if (newKeyPoint.response > (*it).response)
                        {                      // if overlap is >t AND response is higher for new kpt
                            *it = newKeyPoint; // replace old key point with new one
                            break;             // quit loop over keypoints
                        }
                    }
                }
                
            }
        } // eof loop over cols
    }     // eof loop over rows
  	

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
	if (bVis)
	{
		string windowName = "Harris Keypoints";
		cv::Mat visImg = dst_norm_scaled.clone();
		cv::drawKeypoints(dst_norm_scaled,keypoints,visImg,cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		cv::imshow(windowName,visImg);
	}


}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    //// STUDENT ASSIGNMENT
    //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
    //// -> FAST, BRISK, ORB, AKAZE, SIFT
    cv::Ptr<cv::FeatureDetector> detector;
    string windowName;
    double t = (double)cv::getTickCount();
	if (detectorType.compare("FAST")==0)
	{
		int threshold = 100;
		detector = cv::FastFeatureDetector::create(threshold);
		detector->detect(img,keypoints);
//       	cout << keypoints.size() << endl;
		windowName = "FAST features";

	}

	if (detectorType.compare("BRISK")==0)
	{
		detector = cv::BRISK::create();
		detector->detect(img,keypoints);
		windowName = "BRISK features";
	}

	if (detectorType.compare("ORB")==0)
	{
		int nfeatures = 100;
		detector = cv::ORB::create(nfeatures);
		detector->detect(img,keypoints);
		windowName = "ORB features";
	}

	if (detectorType.compare("AKAZE")==0)
	{
		detector = cv::AKAZE::create();
		detector->detect(img,keypoints);
		windowName = "AKAZE features";
	}

	if (detectorType.compare("SIFT")==0)
	{
		detector = cv::xfeatures2d::SIFT::create();
		detector->detect(img,keypoints);
		windowName = "SIFT features";
	}
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
//     cout << detectorType << " with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
	if (bVis)
	{
		cv::Mat visImg = img.clone();
		cv::drawKeypoints(img,keypoints,visImg,cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		cv::imshow(windowName,visImg);
	}
}