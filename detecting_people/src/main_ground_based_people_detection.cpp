/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2013-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * main_ground_based_people_detection_app.cpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 *
 * Example file for performing people detection on a Kinect live stream.
 * As a first step, the ground is manually initialized, then people detection is performed with the GroundBasedPeopleDetectionApp class,
 * which implements the people detection algorithm described here:
 * M. Munaro, F. Basso and E. Menegatti,
 * Tracking people within groups with RGB-D data,
 * In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
 */

#include <opencv2/opencv.hpp>  
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "frmsg/people.h"
#include "frmsg/followme_state.h"
#include <reading_pointcloud/reading_pointcloud.h>
#include <ros/time.h>
//#include <opencv2/opencv.hpp>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

#define NH 10
#define NS 10
#define NV 10
/* max HSV values */
#define H_MAX 360.0
#define S_MAX 1.0
#define V_MAX 1.0

/* low thresholds on saturation and value for histogramming */
#define S_THRESH 0.1
#define V_THRESH 0.2

int MIN_cmp( int a, int b )
{
  if ( a > b )
    return b;
  else
    return a;
}

// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

// Mutex: //
boost::mutex cloud_mutex;


int current_state = 0;
// 0: STOP
// 1: RUN
// 2: RESUME

enum { COLS = 640, ROWS = 480 };

typedef struct histogram {
  float histo[NH*NS + NV];   /**< histogram array */
  int n;                     /**< length of histogram array */
}histogram;

int print_help()
{
  cout << "*******************************************************" << std::endl;
  cout << "Ground based people detection app options:" << std::endl;
  cout << "   --help    <show_this_help>" << std::endl;
  cout << "   --svm     <path_to_svm_file>" << std::endl;
  cout << "   --conf    <minimum_HOG_confidence (default = -1.5)>" << std::endl;
  cout << "   --min_h   <minimum_person_height (default = 1.3)>" << std::endl;
  cout << "   --max_h   <maximum_person_height (default = 2.3)>" << std::endl;
  cout << "*******************************************************" << std::endl;
  return 0;
}

int histo_bin( float h, float s, float v )
{
  int hd, sd, vd;

  /* if S or V is less than its threshold, return a "colorless" bin */
  vd = MIN_cmp( (int)(v * NV / V_MAX), NV-1 );
  if( s < S_THRESH  ||  v < V_THRESH )
    return NH * NS + vd;
  
  /* otherwise determine "colorful" bin */
  hd = MIN_cmp( (int)(h * NH / H_MAX), NH-1 );
  sd = MIN_cmp( (int)(s * NS / S_MAX), NS-1 );
  return sd * NH + hd;
}

int max( int a, int b, int c )
{
  if ( a > b )
  {
    if ( a > c )
    return a;
  else
    return c;
  }
  else
  {
    if ( b > c )
    return b;
  else
    return c;
  }
}

int min( int a, int b, int c )
{
  if ( a < b )
  {
    if ( a < c )
    return a;
  else
    return c;
  }
  else
  {
    if ( b < c )
    return b;
   else
    return c;
  }
}

void rgb2hsv( int r, int g, int b, float& h, float& s, float& v )
{
  int max_ = max( r, g, b );
  int min_ = min( r, g, b );
  v = max_;
  if ( max == 0 )
    s = 0;
  else
  s = ( max_ - min_ ) / (float)max_;
  if ( max_ == min_ )
    h = 0;
  if ( r == max_ )
    h = ( g - b ) / (float)( max_ - min_ ) * 60;
  if ( g == max_ )
    h = 120 + ( b - r ) / (float)( max_ - min_ ) * 60;
  if ( b == max_ )
    h = 240 + ( r - g ) / (float)( max_ - min_ ) * 60;
  if ( h < 0 )
    h += 360;
}

void normalize_histogram( histogram* histo )
{
  float* hist;
  float sum = 0, inv_sum;
  int i, n;

  hist = histo->histo;
  n = histo->n;

  /* compute sum of all bins and multiply each bin by the sum's inverse */
  for( i = 0; i < n; i++ )
    sum += hist[i];
  inv_sum = 1.0 / sum;
  for( i = 0; i < n; i++ )
    hist[i] *= inv_sum;
}

histogram* calc_histogram( PointCloudT::Ptr& cloud )
{
  histogram* histo;
  float* hist;
  int bin;
  float h = 0, s = 0, v = 0;

  histo = (histogram*)malloc( sizeof(histogram) );
  histo->n = NH*NS + NV;
  hist = histo->histo;
  memset( hist, 0, histo->n * sizeof(float) );
  
  for( int i = 0; i < cloud->points.size(); i++ )
  {
    rgb2hsv( (int)cloud->points[i].r, (int)cloud->points[i].g, (int)cloud->points[i].b, h, s, v );
  bin = histo_bin( h, s, v );
  hist[bin] += 1;
  }
  normalize_histogram( histo );
  return histo;
}
histogram* calc_histogram_a( PointCloudT::Ptr& cloud )
{
  histogram* histo;
  float* hist;
  int bin;
  float h = 0, s = 0, v = 0;

  histo = (histogram*)malloc( sizeof(histogram) );
  histo->n = NH*NS + NV;
  hist = histo->histo;
  memset( hist, 0, histo->n * sizeof(float) );
  
  for( int i = 0; i < cloud->points.size(); i++ )
  {
    rgb2hsv( (int)cloud->points[i].r, (int)cloud->points[i].g, (int)cloud->points[i].b, h, s, v );
  bin = histo_bin( h, s, v );
  hist[bin] += 1;
  }
  //normalize_histogram( histo );
  return histo;
}

float histo_dist_sq( histogram* h1, histogram* h2 )
{
  float* hist1, * hist2;
  float sum = 0;
  int i, n;

  n = h1->n;
  hist1 = h1->histo;
  hist2 = h2->histo;

  /*
    According the the Battacharyya similarity coefficient,
    
    D = \sqrt{ 1 - \sum_1^n{ \sqrt{ h_1(i) * h_2(i) } } }
  */
  for( i = 0; i < n; i++ )
    sum += sqrt( hist1[i]*hist2[i] );
  return 1.0 - sum;
}

void cloud_cb_ (const PointCloudT::ConstPtr &callback_cloud, PointCloudT::Ptr& cloud,
    bool* new_cloud_available_flag)
{
  cloud_mutex.lock ();    // for not overwriting the point cloud from another thread
  *cloud = *callback_cloud;
  *new_cloud_available_flag = true;
  cloud_mutex.unlock ();
}

void add_hist( histogram* hist1, histogram* hist2 )
{
  for ( int i = 0; i <  hist1->n; i++ )
  {
    hist1->histo[i] += hist2->histo[i];
  }
}

struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};
  
void
pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
  struct callback_args* data = (struct callback_args *)args;  
  if (event.getPointIndex () == -1)
    return;
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

void stateCallback(const frmsg::followme_state::ConstPtr& state)
{
  if (current_state == state->state)
    return;
  current_state = state->state;
}

int main (int argc, char** argv)
{

  //ROS Initialization
  ros::init(argc, argv, "detecting_people");
  ros::NodeHandle nh;
  ros::Rate rate(13);

  ros::Subscriber state_sub = nh.subscribe("followme_state", 100, &stateCallback);
  ros::Publisher people_pub = nh.advertise<frmsg::people>("followme_people", 100);
  frmsg::people pub_people_;

  CloudConverter* cc_ = new CloudConverter();

  while (!cc_->ready_xyzrgb_)
  {
    ros::spinOnce();
    rate.sleep();
    if (!ros::ok())
    {
      printf("Terminated by Control-c.\n");
      return -1;
    }
  }

  // Input parameter from the .yaml
  std::string package_path_ = ros::package::getPath("detecting_people") + "/";
  cv::FileStorage* fs_ = new cv::FileStorage(package_path_ + "parameters.yml", cv::FileStorage::READ);

  // Algorithm parameters:
  std::string svm_filename = package_path_ + "trainedLinearSVMForPeopleDetectionWithHOG.yaml";
  std::cout << svm_filename << std::endl;

  float min_confidence = -1.5;
  float min_height = 1.3;
  float max_height = 2.3;
  float voxel_size = 0.06;
  Eigen::Matrix3f rgb_intrinsics_matrix;
  rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics
  
  // Read if some parameters are passed from command line:
  pcl::console::parse_argument (argc, argv, "--svm", svm_filename);
  pcl::console::parse_argument (argc, argv, "--conf", min_confidence);
  pcl::console::parse_argument (argc, argv, "--min_h", min_height);
  pcl::console::parse_argument (argc, argv, "--max_h", max_height);


  // Read Kinect live stream:
  PointCloudT::Ptr cloud_people (new PointCloudT);
  cc_->ready_xyzrgb_ = false;
  while ( !cc_->ready_xyzrgb_ )
  {
    ros::spinOnce();
    rate.sleep();
  }
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud = cc_->msg_xyzrgb_;

  // Display pointcloud:
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  // Add point picking callback to viewer:
  struct callback_args cb_args;
  PointCloudT::Ptr clicked_points_3d (new PointCloudT);
  cb_args.clicked_points_3d = clicked_points_3d;
  cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer);
  viewer.registerPointPickingCallback (pp_callback, (void*)&cb_args);
  std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

  // Spin until 'Q' is pressed:
  viewer.spin();
  std::cout << "done." << std::endl;
  
  //cloud_mutex.unlock ();    

  // Ground plane estimation:
  Eigen::VectorXf ground_coeffs;
  ground_coeffs.resize(4);
  std::vector<int> clicked_points_indices;
  for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
    clicked_points_indices.push_back(i);
  pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
  model_plane.computeModelCoefficients(clicked_points_indices,ground_coeffs);
  std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;

  // Initialize new viewer:
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");          // viewer initialization
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  // Create classifier for people detection:  
  pcl::people::PersonClassifier<pcl::RGB> person_classifier;
  person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

  // People detection app initialization:
  pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
  people_detector.setVoxelSize(voxel_size);                        // set the voxel size
  people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
  people_detector.setClassifier(person_classifier);                // set person classifier
  people_detector.setHeightLimits(min_height, max_height);         // set person classifier
//  people_detector.setSensorPortraitOrientation(true);             // set sensor orientation to vertical

  // For timing:
  static unsigned count = 0;
  static double last = pcl::getTime ();
  
  int people_count = 0;
  histogram* first_hist;

  int max_people_num = (int)fs_->getFirstTopLevelNode()["max_people_num"];
  // Main loop:
  while (!viewer.wasStopped() && ros::ok() )
  {
    if ( current_state == 1 )
    {
      if ( cc_->ready_xyzrgb_ )    // if a new cloud is available
      {

        std::vector<float> x;
        std::vector<float> y;
        std::vector<float> depth;

        cloud = cc_->msg_xyzrgb_;
        PointCloudT::Ptr cloud_new(new PointCloudT(*cloud));
        cc_->ready_xyzrgb_ = false;

        // Perform people detection on the new cloud:
        std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
        people_detector.setInputCloud(cloud_new);
        people_detector.setGround(ground_coeffs);                    // set floor coefficients
        people_detector.compute(clusters);                           // perform people detection

        ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

        // Draw cloud and people bounding boxes in the viewer:
        viewer.removeAllPointClouds();
        viewer.removeAllShapes();
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
        viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
        unsigned int k = 0;
        std::vector<pcl::people::PersonCluster<PointT> >::iterator it;
        std::vector<pcl::people::PersonCluster<PointT> >::iterator it_min;

        float min_z = 10.0;

        float histo_dist_min = 2.0;
        int id = -1;

        for(it = clusters.begin(); it != clusters.end(); ++it)
        {
          if(it->getPersonConfidence() > min_confidence)             // draw only people with confidence above a threshold
          {

            x.push_back((it->getTCenter())[0]);
            y.push_back((it->getTCenter())[1]);
            depth.push_back(it->getDistance());
            // draw theoretical person bounding box in the PCL viewer:
            /*pcl::copyPointCloud( *cloud, it->getIndices(), *cloud_people);
            if ( people_count == 0 )
            {
              first_hist = calc_histogram_a( cloud_people );
              people_count++;
              it->drawTBoundingBox(viewer, k);
            }
            else if ( people_count <= 10 )
            {
              histogram* hist_tmp = calc_histogram_a( cloud_people );
              add_hist( first_hist, hist_tmp );
              it->drawTBoundingBox(viewer, k);
              people_count++;
            }*/
            pcl::copyPointCloud( *cloud, it->getIndices(), *cloud_people);
            if ( people_count < 11 )
            {
              if ( it->getDistance() < min_z )
              {
                it_min = it;
                min_z = it->getDistance();
              }
            }
            else if ( people_count == 11 )
            {
              normalize_histogram( first_hist );
              people_count++;
              histogram* hist_tmp = calc_histogram( cloud_people );
              float tmp = histo_dist_sq( first_hist, hist_tmp );
              std::cout << "The histogram distance is " << tmp << std::endl;
              histo_dist_min = tmp;
              it_min = it;
              id = k;
            }
            else
            {
              histogram* hist_tmp = calc_histogram( cloud_people );
              float tmp = histo_dist_sq( first_hist, hist_tmp );
              std::cout << "The histogram distance is " << tmp << std::endl;
              if ( tmp < histo_dist_min )
              {
                histo_dist_min = tmp;
                it_min = it;
                id = k;
              }
            }
            k++;
            //std::cout << "The data of the center is " << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].x << "  " << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].y << " " << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << std::endl;
            //std::cout << "The size of the people cloud is " <<  cloud_people->points.size() << std::endl;
            std::cout << "The " << k << " person's distance is " << it->getDistance() << std::endl;
          }
        }
        pub_people_.x = x;
        pub_people_.y = y;
        pub_people_.depth = depth;
        if ( k > 0 )
        {
          if ( people_count <= 11 )
          {
            pcl::copyPointCloud( *cloud, it_min->getIndices(), *cloud_people);
            if ( people_count == 0)
            {
              first_hist = calc_histogram_a( cloud_people );
              people_count++;
              it_min->drawTBoundingBox(viewer, 1);
            }
            else if ( people_count < 11 )
            {
              histogram* hist_tmp = calc_histogram_a( cloud_people );
              add_hist( first_hist, hist_tmp );
              it_min->drawTBoundingBox(viewer, 1);
              people_count++;
            }
          }
          else
          {
            pub_people_.id = k-1;
            if ( histo_dist_min < 1.3 )
            {
              it_min->drawTBoundingBox(viewer, 1);
              std::cout << "The minimum distance of the histogram is " << histo_dist_min << std::endl;
              std::cout << "The vector is " << it_min->getTCenter() << std::endl << "while the elements are " << (it_min->getTCenter())[0] << " " << (it_min->getTCenter())[1] << std::endl;
            }
            else
            {
              pub_people_.id = -1;
            }
          }
        }
        else
        {
          pub_people_.id = -1;
        }
        pub_people_.header.stamp = ros::Time::now();
        people_pub.publish(pub_people_);
        
        std::cout << k << " people found" << std::endl;
        viewer.spinOnce();
        ros::spinOnce();

        // Display average framerate:
        if (++count == 30)
        {
          double now = pcl::getTime ();
          std::cout << "Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
          count = 0;
          last = now;
        }
        //cloud_mutex.unlock ();
      }
    }
  }

  return 0;
}