#include "ros/ros.h"
#include "sift/stdafx.h"
#include <time.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "reading_image/reading_image.h"
#include "sift/sift.h"
#include "sift/minpq.h"
#include "sift/kdtree.h"
#include <ros/package.h>

IplImage* stack_imgs( IplImage* img1, IplImage* img2 )
{
	IplImage* stacked = cvCreateImage( cvSize( MAX(img1->width, img2->width), img1->height + img2->height ), IPL_DEPTH_8U, 3 );
	cvZero( stacked );
	cvSetImageROI( stacked, cvRect( 0, 0, img1->width, img1->height ) );
	cvAdd( img1, stacked, stacked, NULL );
	cvSetImageROI( stacked, cvRect(0, img1->height, img2->width, img2->height) );
	cvAdd( img2, stacked, stacked, NULL );
	cvResetImageROI( stacked );
//	cvShowImage( "stack", stacked );
	return stacked;
}

int main( int argc, char** argv )
{
	int KDTREE_BBF_MAX_NN_CHKS = 200;
	float NN_SQ_DIST_RATIO_THR = 0.49;
	const CvScalar color = cvScalar( 255, 255, 0 );
	ros::init(argc, argv, "SIFT");
	ros::NodeHandle n;
	ros::Rate rate(33);

	ImageConverter ic;

	while ( !ic.ready )
	{
		ros::spinOnce();
		rate.sleep();
		if ( !ros::ok() )
		{
			printf("terminated by control_c\n");
			return 0;
		}
	}

	string filepath = ros::package::getPath("sift") + "/";

	ifstream fin( (filepath + "store.txt").data(), ios::in); 
	//ifstream fin_main_pic( (filepath + "main_pic.txt").data(), ios::in);
	int pic_num = 5;
	string find;
	//cout << "how many pictures?" << endl;
	//cin >> pic_num;
	//cout << "which picture?" << endl;
	//cin >> find;

	time_t rawtime; 
	struct tm * timeinfo; 
	time ( &rawtime ); 
	timeinfo = localtime ( &rawtime ); 
	printf ( "The current date/time is: %s", asctime (timeinfo) ); 

	char line[1024] = {0}; 
	string* store = new string [pic_num+1];
	string main_pic_name;
	int pic = 0;
	int find_pic = 0;
	while(fin.getline(line, sizeof(line))) 
	{
		stringstream word(line); 
		word >> store[pic];
		store[pic] = filepath + store[pic];
		/*if (store[pic] == find)
		{
			cout << store[pic] << endl;
			find_pic = pic;
		}*/
		pic++;
	}
	//fin_main_pic.getline(line, sizeof(line));
	//stringstream word(line);
	//word >> main_pic_name;
	//fin_main_pic.clear();
	//fin_main_pic.close();
	fin.clear(); 
	fin.close();

	IplImage* img;
	//IplImage* img1;
	struct feature* features;//, * features1;

	feature** features_all = new feature*[pic];
	int* features_num = new int[pic];
	for (int i = 0; i < pic; i++ )
		features_num[i] = 0;
	IplImage** img_all = new IplImage*[pic];
	for ( int i = 0; i < pic; i++ )
	{
		//printf ( "Finding features in template picture %d\n", i );
		img_all[i] = cvLoadImage( store[i].data(), 1 );
		features_num[i] = sift_features( img_all[i], &features_all[i] );
		printf ( "%d features in template picture %d\n", features_num[i], i );
		time ( &rawtime ); 
		timeinfo = localtime ( &rawtime ); 
		printf ( "The current date/time is: %s", asctime (timeinfo) );
	}
/*
	printf ( "Finding features in main picture\n" );
	img = cvLoadImage( main_pic_name.data(), 1 );
	int n1 = sift_features( img, &features );
	printf ( "%d features in main picture\n", n1 );
*/
	
	//cvShowImage( "main", img );
	//for (int i = 0; i < n1; i++)
		//cvCircle( img, cvPoint(features[i].x, features[i].y), 5, color, 1, 8, 0 );
	//cvShowImage( "Foundmain", img );


	//cvShowImage( "template", img1 );
	//for (int i = 0; i < n2; i++)
		//cvCircle( img1, cvPoint(features1[i].x, features1[i].y), 5, color, 1, 8, 0 );
	//cvShowImage( "Foundtemplate", img1 );

	bool features_catched = false;
	while ( ros::ok() )
	{
		if ( ic.ready == true )
		{
			ic.ready = false;
			*img = ic.curr_image;
			int n1 = sift_features( img, &features );
			printf ( "%d features in main picture\n", n1 );
			time ( &rawtime ); 
			timeinfo = localtime ( &rawtime ); 
			printf ( "The current date/time is: %s", asctime (timeinfo) );
			features_catched = false;
			for ( int j = 0; j < pic ; j++ )
			{
				IplImage* stacked;
				IplImage* ransac;
				struct feature* feat;
				struct feature** nbrs;
				struct kd_node* kd_root;
				CvPoint pt1, pt2;
				double d0, d1;
				int k, i, m = 0;
				CvMat point1_test;
				CvMat point2_test;
				double point1[3];
				double point2[3] = { 0 };

				stacked = stack_imgs( img, img_all[j] );
				ransac = stack_imgs( img, img_all[j] );

				kd_root = kdtree_build( features_all[j], features_num[j] );
				for( i = 0; i < n1; i++ )
				{
					feat = features + i;
					k = kdtree_bbf_knn( kd_root, feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS );
					if( k == 2 )
					{
						d0 = descr_dist_sq( feat, nbrs[0] );
						d1 = descr_dist_sq( feat, nbrs[1] );
						if( d0 < d1 * NN_SQ_DIST_RATIO_THR )
						{
							//pt1 = cvPoint( cvRound( feat->x ), cvRound( feat->y ) );
							//pt2 = cvPoint( cvRound( nbrs[0]->x ), cvRound( nbrs[0]->y ) );
							//pt2.y += img->height;
							//cvCircle( stacked, pt1, 3, cvScalar( (i*10)%255, (i*10)%255, 127 ), 1, 8, 0 );
							//cvCircle( stacked, pt2, 3, cvScalar( (i*10)%255, (i*10)%255, 127 ), 1, 8, 0 );
							//cvLine( stacked, pt1, pt2, cvScalar( (i*10)%255, (i*10)%255, 127 ), 1, 8, 0 );
							m++;
							features[i].fwd_match = nbrs[0];
						}
					}
					free( nbrs );
				}

				double accounts = m * 100 / (double)features_num[j];
				printf( "%d total matches, accounts for %f %%, in pic %d\n", m, accounts, j);
				//cvNamedWindow( "Matches", 1 );
				//cvShowImage( "Matches", stacked );

				time ( &rawtime ); 
				timeinfo = localtime ( &rawtime ); 
				printf ( "The current date/time is: %s", asctime (timeinfo) );	

				CvMat* H = ransac_xform( features, n1, FEATURE_FWD_MATCH, lsq_homog, 4, 0.01, error, 2, NULL, NULL );
			    if( H )
			    {
					for( i = 0; i < n1; i++ )
					{
						feat = features + i;
						k = kdtree_bbf_knn( kd_root, feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS );
						if( k == 2 )
						{
							d0 = descr_dist_sq( feat, nbrs[0] );
							d1 = descr_dist_sq( feat, nbrs[1] );
							if( d0 < d1 * NN_SQ_DIST_RATIO_THR )
							{
								pt1 = cvPoint( cvRound( feat->x ), cvRound( feat->y ) );
								pt2 = cvPoint( cvRound( nbrs[0]->x ), cvRound( nbrs[0]->y ) );
								pt2.y += img->height;
								point1[0] = pt1.x;
								point1[1] = pt1.y;
								point1[2] = 1.0;
								cvInitMatHeader( &point1_test, 3, 1, CV_64FC1, point1, CV_AUTOSTEP );
								cvInitMatHeader( &point2_test, 3, 1, CV_64FC1, point2, CV_AUTOSTEP );
								cvMatMul( H, &point1_test, &point2_test );
								/*if ( abs( point2[0]/point2[2]-pt2.x) < 2 && abs( point2[1]/point2[2]+img->height-pt2.y) < 2 )
								{
									cvCircle( ransac, pt1, 3, cvScalar( (i*10)%255, (i*10)%255, 127 ), 1, 8, 0 );
									cvCircle( ransac, pt2, 3, cvScalar( (i*10)%255, (i*10)%255, 127 ), 1, 8, 0 );
									cvLine( ransac, pt1, pt2, cvScalar( (i*10)%255, (i*10)%255, 127 ), 1, 8, 0 );
								}*/
			//					features[i].fwd_match = nbrs[0];
							}
						}
						free( nbrs );
						//printf("features catched, going to exit\n");
					}

					//cvNamedWindow( "Xformed" );
					//cvShowImage( "Xformed", ransac );

					features_catched = true;
					time ( &rawtime ); 
					timeinfo = localtime ( &rawtime ); 
					printf ( "ransac.. The current date/time is: %s", asctime (timeinfo) ); 
			    }
				//cvWaitKey( 0 );
				cvReleaseImage( &ransac );
				cvReleaseMat( &H );
				//cvDestroyWindow( "main" );
				//cvDestroyWindow( "Foundmain" );
				//cvDestroyWindow( "template" );
				//cvDestroyWindow( "Foundtemplate" );
				//cvReleaseImage( &img_all[j] );
				cvReleaseImage( &stacked );
				kdtree_release( kd_root );
			}
			if (!features_catched)
			{
				printf("Sorry, there is no item in the picture\n");
			}
			else
			{
				printf("Item catched in the picture!\n");
			}
		}
		ros::spinOnce();
		rate.sleep();
	}
	//cvReleaseImage( &img );
	free( features );
	for ( int i = 0; i < pic; i++ )
	{
		free( features_all[i] );
	}
	free(features_all);
	return 0;
}

