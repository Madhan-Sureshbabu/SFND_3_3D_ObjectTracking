
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <math.h>

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
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

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

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
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
        int top=1e2, left=1e2, bottom=0.0, right=0.0; 
        float xwmin=1e2, ywmin=1e2, ywmax=-1e2;
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
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 0.5, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 0.5, currColor);  
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
    double dist_mean=0, var = 0;
  	cv::Point prev_pt, curr_pt;
  	vector<double> distances;
    for (auto it1 = kptMatches.begin(); it1!=kptMatches.end(); ++it1)
    {
      cv::Point2f pt = cv::Point2f(kptsCurr[it1->trainIdx].pt);
      if (boundingBox.roi.contains(pt))
      {            
        	prev_pt = kptsPrev[it1->queryIdx].pt;
        	curr_pt = kptsCurr[it1->trainIdx].pt;
        	distances.push_back( sqrt(pow( prev_pt.x - curr_pt.x ,2) + pow( prev_pt.y - curr_pt.y ,2)) );
      }
    }
  	dist_mean = std::accumulate(distances.begin(), distances.end(),0.0) / kptMatches.size();
  	
    for (double d : distances)
    {
		var += pow(d - dist_mean, 2);
    }
  
  	for (int i=0; i<kptMatches.size(); i++)
    {
      	cv::Point2f pt = cv::Point2f(kptsCurr[kptMatches[i].trainIdx].pt);
        if (boundingBox.roi.contains(pt))
        {            
          if (distances[i] - dist_mean < 0.25 * sqrt(var) )
          {
            boundingBox.kptMatches.push_back(kptMatches[i]);
          }
        }
    }
  
//   	for (auto it1 = boundingBox.kptMatches.begin(); it1!=boundingBox.kptMatches.end(); ++it1)
//     {
		
//     }
//   	dist_mean /= boundingBox.kptMatches.size();
//   	double ratio;
//   	for (auto it1 = boundingBox.kptMatches.begin(); it1!=boundingBox.kptMatches.end(); ++it1)
//     {
// 		ratio = abs(it1->distance-dist_mean)/dist_mean;
//       	if (ratio>0.2)
//           boundingBox.kptMatches.erase(it1);
//     }
//   	cout<<"Number of matches in bounding box "<<boundingBox.kptMatches.size() << endl;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{

  	cv::Point prev_pt_outer, curr_pt_outer, prev_pt_inner, curr_pt_inner;
  	double dist_prev, dist_curr, ratio;
  	vector<double> dist_ratio_list;
    for (auto it1 = kptMatches.begin(); it1!=kptMatches.end(); it1++)
    {
      	prev_pt_outer = kptsPrev[it1->queryIdx].pt;
// 		prev_pts.push_back(pt);
      
      	curr_pt_outer = kptsCurr[it1->trainIdx].pt;
//       	curr_pts.push_back(pt);
      
      	for (auto it2 = it1 + 1; it2!=kptMatches.end(); it2++)
        {
			prev_pt_inner = kptsPrev[it2->queryIdx].pt;
          	curr_pt_inner = kptsCurr[it2->trainIdx].pt;
          
          	dist_prev = cv::norm(prev_pt_outer-prev_pt_inner);
          	dist_curr = cv::norm(curr_pt_outer-curr_pt_inner);
          
          	if (dist_prev > std::numeric_limits<double>::epsilon() && dist_curr > dist_prev )
            {
              ratio = dist_curr/dist_prev;
              dist_ratio_list.push_back(ratio);
            }
        }
    }
  
  	if (dist_ratio_list.size() == 0)
      TTC = NAN;
    else
    {
      std::sort(dist_ratio_list.begin(), dist_ratio_list.end());
      int size = dist_ratio_list.size();
      double median_ratio;
      if (size%2 == 0)
        median_ratio = (dist_ratio_list[size/2] + dist_ratio_list[(size/2)-1])/2;
      else
        median_ratio = dist_ratio_list[floor(size/2)];

      TTC = -1 / (frameRate * (1 - median_ratio) ) ;
    }

}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
  
  double d0=0,d1=0;
  for (auto it=lidarPointsPrev.begin(); it!=lidarPointsPrev.end(); it++)
   	 d0 += it->x;
  d0 /= lidarPointsPrev.size();
  
  for (auto it=lidarPointsCurr.begin(); it!=lidarPointsCurr.end(); it++)
    d1 += it->x;
  d1 /= lidarPointsCurr.size();
  
  TTC = d1 / ((d0 - d1)*frameRate);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
  int numBoundingBoxes_prevFrame = prevFrame.boundingBoxes.size();
  int numBoundingBoxes_currFrame = currFrame.boundingBoxes.size();
  int prev_curr_Matchcount[numBoundingBoxes_prevFrame][numBoundingBoxes_currFrame] = {0};
//   cout << "numBoundingBoxes_prevFrame : " << numBoundingBoxes_prevFrame << " numBoundingBoxes_currFrame : " << numBoundingBoxes_currFrame << endl;
  vector<int> boundingBoxMatch_prevFrame, boundingBoxMatch_currFrame;
  
  for (int i=0; i<numBoundingBoxes_prevFrame; i++)
  {
    for (int j=0; j<numBoundingBoxes_currFrame; j++)
    {
		prev_curr_Matchcount[i][j] = 0;
    }
  }
  
//   for (int i=0; i<numBoundingBoxes_prevFrame; i++)
//   {
//     for (int j=0; j<numBoundingBoxes_currFrame; j++)
//     {
// 		cout << prev_curr_Matchcount[i][j] << " " ;
//     }
//     cout << endl;
//   }
//   cout << "Num matches " << matches.size() << endl;
  for (auto it1 = matches.begin(); it1!=matches.end(); ++it1)
  {
  	cv::KeyPoint prev_kpt = prevFrame.keypoints[it1->queryIdx];
    cv::KeyPoint curr_kpt = currFrame.keypoints[it1->trainIdx];
    cv::Point prev_pt = cv::Point(prev_kpt.pt.x,prev_kpt.pt.y);
    cv::Point curr_pt = cv::Point(curr_kpt.pt.x,curr_kpt.pt.y);
    boundingBoxMatch_prevFrame.clear();
    boundingBoxMatch_currFrame.clear();
    
    for (int i=0; i<numBoundingBoxes_prevFrame; i++)
    {
     	if (prevFrame.boundingBoxes[i].roi.contains(prev_pt))
        {
			boundingBoxMatch_prevFrame.push_back(i);
        }
    }
    
    for (int j=0; j<numBoundingBoxes_currFrame; j++)
    {
		if (currFrame.boundingBoxes[j].roi.contains(curr_pt))
        {
			boundingBoxMatch_currFrame.push_back(j);
        }
    }
    
    for (int i : boundingBoxMatch_prevFrame)
    {
		for (int j : boundingBoxMatch_currFrame)
        {
			prev_curr_Matchcount[i][j] += 1;
        }
    }
    
  }
  
//   for (int i=0; i<numBoundingBoxes_prevFrame; i++)
//   {
//     for (int j=0; j<numBoundingBoxes_currFrame; j++)
//     {
// 		cout << prev_curr_Matchcount[i][j] << " " ;
//     }
//     cout << endl;
//   }
  
  int best_count, best_id, boxId_prev, boxId_curr;
  
  for (int i=0; i<numBoundingBoxes_prevFrame; i++)
  {
  		best_count = -1;
  		best_id = -1;
		for (int j=0; j<numBoundingBoxes_currFrame; j++)
        {
			if (prev_curr_Matchcount[i][j] > best_count)
            {
				best_count = prev_curr_Matchcount[i][j];
  				best_id = j;
              	boxId_prev = prevFrame.boundingBoxes[i].boxID;
                boxId_curr = currFrame.boundingBoxes[j].boxID;
            }	
        }
  			bbBestMatches[prevFrame.boundingBoxes[i].boxID] =  currFrame.boundingBoxes[best_id].boxID;
  }

  
}
  
