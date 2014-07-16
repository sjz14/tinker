#ifndef SIFT_H
#define SIFT_H

#include "cxcore.h"
#include <time.h>
#include "features.h"
#include <cv.h>
#include <highgui.h>
#include <iostream>
using namespace std;

struct detection_data
{
  int r;
  int c;
  int octv;
  int intvl;
  double subintvl;
  double scl_octv;
};

#define RANSAC_ERR_TOL 5
#define RANSAC_INLIER_FRAC_EST 0.25
#define RANSAC_PROB_BAD_SUPP 0.10
#define feat_ransac_data( feat ) ( (struct ransac_data*) (feat)->feature_data )
#define SIFT_INTVLS 5
#define SIFT_SIGMA 1.6
#define SIFT_CONTR_THR 0.04
#define SIFT_CURV_THR 10
#define SIFT_IMG_DBL 1
#define SIFT_DESCR_WIDTH 4
#define SIFT_DESCR_HIST_BINS 8
#define SIFT_INIT_SIGMA 0.5
#define SIFT_IMG_BORDER 5
#define SIFT_MAX_INTERP_STEPS 5
#define SIFT_ORI_HIST_BINS 36
#define SIFT_ORI_SIG_FCTR 1.5
#define SIFT_ORI_RADIUS 3.0 * SIFT_ORI_SIG_FCTR
#define SIFT_ORI_SMOOTH_PASSES 2
#define SIFT_ORI_PEAK_RATIO 0.8
#define SIFT_DESCR_SCL_FCTR 3.0
#define SIFT_DESCR_MAG_THR 0.2
#define SIFT_INT_DESCR_FCTR 512.0
#define feat_detection_data(f) ( (struct detection_data*)(f->feature_data) )


typedef double (*ransac_err_fn)( CvPoint2D64f pt, CvPoint2D64f mpt, CvMat* T );
typedef CvMat* (*ransac_xform_fn)( CvPoint2D64f* pts, CvPoint2D64f* mpts, int n );
int sift_features( IplImage* img, struct feature** feat );
int _sift_features( IplImage* img, struct feature** feat, int intvls, double sigma, double contr_thr, int curv_thr, int img_dbl, int descr_width, int descr_hist_bins );
int pixval8( IplImage* img, int r, int c ) { return (int)( ( (uchar*)(img->imageData + img->widthStep*r) )[c] ); }
void setpix8( IplImage* img, int r, int c, uchar val) { ( (uchar*)(img->imageData + img->widthStep*r) )[c] = val; }
float pixval32f( IplImage* img, int r, int c ) { return ( (float*)(img->imageData + img->widthStep*r) )[c]; }
void setpix32f( IplImage* img, int r, int c, float val ) { ( (float*)(img->imageData + img->widthStep*r) )[c] = val; }
double pixval64f( IplImage* img, int r, int c ) { return (double)( ( (double*)(img->imageData + img->widthStep*r) )[c] ); }
void setpix64f( IplImage* img, int r, int c, double val ) { ( (double*)(img->imageData + img->widthStep*r) )[c] = val; }

IplImage* create_init_img( IplImage*, int, double );
IplImage* convert_to_gray32( IplImage* );
IplImage*** build_gauss_pyr( IplImage*, int, int, double );
IplImage* downsample( IplImage* );
IplImage*** build_dog_pyr( IplImage***, int, int );
CvSeq* scale_space_extrema( IplImage***, int, int, double, int, CvMemStorage* );
bool is_extremum( IplImage***, int, int, int, int );
struct feature* interp_extremum( IplImage***, int, int, int, int, int, double );
void interp_step( IplImage***, int, int, int, int, double*, double*, double* );
CvMat* deriv_3D( IplImage***, int, int, int, int );
CvMat* hessian_3D( IplImage***, int, int, int, int );
double interp_contr( IplImage***, int, int, int, int, double, double, double );
struct feature* new_feature( void );
int is_too_edge_like( IplImage*, int, int, int );
void calc_feature_scales( CvSeq*, double, int );
void adjust_for_img_dbl( CvSeq* );
void calc_feature_oris( CvSeq*, IplImage*** );
double* ori_hist( IplImage*, int, int, int, int, double );
int calc_grad_mag_ori( IplImage*, int, int, double*, double* );
void smooth_ori_hist( double*, int );
double dominant_ori( double*, int );
void add_good_ori_features( CvSeq*, double*, int, double, struct feature* );
struct feature* clone_feature( struct feature* );
void compute_descriptors( CvSeq*, IplImage***, int, int );
double*** descr_hist( IplImage*, int, int, double, double, int, int );
void interp_hist_entry( double***, double, double, double, double, int, int );
void hist_to_descr( double***, int, int, struct feature* );
void normalize_descr( struct feature* );
int feature_cmp( void*, void*, void* );
void release_descr_hist( double****, int );
void release_pyr( IplImage****, int, int );

int sift_features( IplImage* img, struct feature** feat )
{
  return _sift_features( img, feat, SIFT_INTVLS, SIFT_SIGMA, SIFT_CONTR_THR, SIFT_CURV_THR, SIFT_IMG_DBL, SIFT_DESCR_WIDTH, SIFT_DESCR_HIST_BINS );
}

int _sift_features( IplImage* img, struct feature** feat, int intvls, double sigma, double contr_thr, int curv_thr, int img_dbl, int descr_width, int descr_hist_bins )
{
  CvMemStorage* storage = cvCreateMemStorage( 0 );
 
  IplImage* init_img = create_init_img( img, img_dbl, sigma );
  int octvs = log( (long double)MIN( init_img->width, init_img->height ) ) / log((float)2) - 3;
  IplImage*** gauss_pyr = build_gauss_pyr( init_img, octvs, intvls, sigma );
  IplImage*** dog_pyr = build_dog_pyr( gauss_pyr, octvs, intvls );
  
  CvSeq* features = scale_space_extrema( dog_pyr, octvs, intvls, contr_thr, curv_thr, storage );
  calc_feature_scales( features, sigma, intvls );
  if( img_dbl )
    adjust_for_img_dbl( features );
  calc_feature_oris( features, gauss_pyr );
  compute_descriptors( features, gauss_pyr, descr_width, descr_hist_bins );

  cvSeqSort( features, (CvCmpFunc)feature_cmp, NULL );
  int n = features->total;
  *feat = ( feature* )calloc( n, sizeof(struct feature) );
  *feat = ( feature* )cvCvtSeqToArray( features, *feat, CV_WHOLE_SEQ );
  for( int i = 0 ; i < n; i++ ) {
      free( (*feat)[i].feature_data );
      (*feat)[i].feature_data = NULL;
  }
  
  cvReleaseMemStorage( &storage );
  cvReleaseImage( &init_img );
  release_pyr( &gauss_pyr, octvs, intvls + 3 );
  release_pyr( &dog_pyr, octvs, intvls + 2 );
  return n;
}

IplImage* create_init_img( IplImage* img, int img_dbl, double sigma )
{
  IplImage* dbl;
  double sig_diff;

  IplImage* gray = convert_to_gray32( img );
  if( img_dbl )
    {
      sig_diff = sqrt( sigma * sigma - SIFT_INIT_SIGMA * SIFT_INIT_SIGMA * 4 );
      dbl = cvCreateImage( cvSize( img->width*2, img->height*2 ), IPL_DEPTH_32F, 1 );
      cvResize( gray, dbl, CV_INTER_CUBIC );
      cvSmooth( dbl, dbl, CV_GAUSSIAN, 0, 0, sig_diff, sig_diff );
      cvReleaseImage( &gray );
      return dbl;
    }
  else
    {
      sig_diff = sqrt( sigma * sigma - SIFT_INIT_SIGMA * SIFT_INIT_SIGMA );
      cvSmooth( gray, gray, CV_GAUSSIAN, 0, 0, sig_diff, sig_diff );
      return gray;
    }
}

IplImage* convert_to_gray32( IplImage* img )
{
  IplImage* gray8;
  IplImage* gray32 = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );
  if( img->nChannels == 1 )
    gray8 = (IplImage*)cvClone( img );
  else {
      gray8 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );
      cvCvtColor( img, gray8, CV_BGR2GRAY );
  }
  cvConvertScale( gray8, gray32, 1.0 / 255.0, 0 );
  cvReleaseImage( &gray8 );
  return gray32;
}

IplImage*** build_gauss_pyr( IplImage* base, int octvs, int intvls, double sigma )
{
  IplImage*** gauss_pyr;
  const int _intvls = intvls;
  double* sig = new double [intvls+3];
  double sig_total, sig_prev, k;
  int i, o;

  gauss_pyr = ( IplImage*** )calloc( octvs, sizeof( IplImage** ) );
  for( i = 0; i < octvs; i++ )
    gauss_pyr[i] = ( IplImage** )calloc( intvls + 3, sizeof( IplImage *) );

  sig[0] = sigma;
  k = pow( 2.0, 1.0 / intvls );
  for( i = 1; i < intvls + 3; i++ )
    {
      sig_prev = pow( k, i - 1 ) * sigma;
      sig_total = sig_prev * k;
      sig[i] = sqrt( sig_total * sig_total - sig_prev * sig_prev );
    }

  for( o = 0; o < octvs; o++ )
    for( i = 0; i < intvls + 3; i++ )
    {
	if( o == 0  &&  i == 0 )
	  gauss_pyr[o][i] = cvCloneImage(base);
	else if( i == 0 )
	  gauss_pyr[o][i] = downsample( gauss_pyr[o-1][intvls] );
	else
	  {
	    gauss_pyr[o][i] = cvCreateImage( cvGetSize(gauss_pyr[o][i-1]), IPL_DEPTH_32F, 1 );
	    cvSmooth( gauss_pyr[o][i-1], gauss_pyr[o][i], CV_GAUSSIAN, 0, 0, sig[i], sig[i] );
	  }
    }

  return gauss_pyr;
}

IplImage* downsample( IplImage* img )
{
  IplImage* half = cvCreateImage( cvSize(img->width / 2, img->height / 2), img->depth, img->nChannels );
  cvResize( img, half, CV_INTER_NN );

  return half;
}

IplImage*** build_dog_pyr( IplImage*** gauss_pyr, int octvs, int intvls )
{
  IplImage*** dog_pyr = ( IplImage*** )calloc( octvs, sizeof( IplImage** ) );
  for( int i = 0; i < octvs; i++ )
    dog_pyr[i] = ( IplImage** )calloc( intvls + 2, sizeof(IplImage*) );

  for( int o = 0; o < octvs; o++ ) {
    for( int i = 0; i < intvls + 2; i++ ) {
      dog_pyr[o][i] = cvCreateImage( cvGetSize(gauss_pyr[o][i]), IPL_DEPTH_32F, 1 );
	  cvSub( gauss_pyr[o][i+1], gauss_pyr[o][i], dog_pyr[o][i], NULL );
    }
  }

  return dog_pyr;
}

CvSeq* scale_space_extrema( IplImage*** dog_pyr, int octvs, int intvls, double contr_thr, int curv_thr, CvMemStorage* storage )
{
  CvSeq* features;
  double prelim_contr_thr = 0.5 * contr_thr / intvls;
  struct feature* feat;
  struct detection_data* ddata;
  int o, i, r, c;

  features = cvCreateSeq( 0, sizeof(CvSeq), sizeof(struct feature), storage );
  for( o = 0; o < octvs; o++ )
    for( i = 1; i <= intvls; i++ )
      for(r = SIFT_IMG_BORDER; r < dog_pyr[o][0]->height-SIFT_IMG_BORDER; r++)
	    for(c = SIFT_IMG_BORDER; c < dog_pyr[o][0]->width-SIFT_IMG_BORDER; c++)
		  if( abs( pixval32f( dog_pyr[o][i], r, c ) ) > prelim_contr_thr )
		    if( is_extremum( dog_pyr, o, i, r, c ) ) {
			  feat = interp_extremum(dog_pyr, o, i, r, c, intvls, contr_thr);
			  if( feat ) {
				ddata = feat_detection_data( feat );
				if( ! is_too_edge_like( dog_pyr[ddata->octv][ddata->intvl], ddata->r, ddata->c, curv_thr ) ) { cvSeqPush( features, feat ); }
				else free( ddata );
				free( feat );
			  }
			}
  
  return features;
}

bool is_extremum( IplImage*** dog_pyr, int octv, int intvl, int r, int c )
{
  double val = pixval32f( dog_pyr[octv][intvl], r, c );

  if( val > 0 ) {
	for( int i = -1; i <= 1; i++ )
	  for( int j = -1; j <= 1; j++ )
		for( int k = -1; k <= 1; k++ )
			if( val < pixval32f( dog_pyr[octv][intvl+i], r + j, c + k ) )
			return false;
  }

  else {
    for( int i = -1; i <= 1; i++ )
      for( int j = -1; j <= 1; j++ )
	    for( int k = -1; k <= 1; k++ )
		  if( val > pixval32f( dog_pyr[octv][intvl+i], r + j, c + k ) )
	      return false;
  }

  return true;
}

struct feature* interp_extremum( IplImage*** dog_pyr, int octv, int intvl, int r, int c, int intvls, double contr_thr )
{
  struct feature* feat;
  struct detection_data* ddata;
  double xi, xr, xc, contr;
  int i;

  for( i = 0; i < SIFT_MAX_INTERP_STEPS; i++ ) {
	interp_step( dog_pyr, octv, intvl, r, c, &xi, &xr, &xc );
    if( abs( xi ) < 0.5  &&  abs( xr ) < 0.5  &&  abs( xc ) < 0.5 )
	break;
      
    c += cvRound( xc );
    r += cvRound( xr );
    intvl += cvRound( xi );
      
    if( intvl < 1  || intvl > intvls  || c < SIFT_IMG_BORDER  || r < SIFT_IMG_BORDER  || c >= dog_pyr[octv][0]->width - SIFT_IMG_BORDER ||  r >= dog_pyr[octv][0]->height - SIFT_IMG_BORDER )
	  return NULL;
  }
 
  if( i >= SIFT_MAX_INTERP_STEPS ) return NULL;
  
  contr = interp_contr( dog_pyr, octv, intvl, r, c, xi, xr, xc );
  if( abs( contr ) < contr_thr / intvls ) return NULL;

  feat = new_feature();
  ddata = feat_detection_data( feat );
  feat->img_pt.x = feat->x = ( c + xc ) * pow( 2.0, octv );
  feat->img_pt.y = feat->y = ( r + xr ) * pow( 2.0, octv );
  ddata->r = r;
  ddata->c = c;
  ddata->octv = octv;
  ddata->intvl = intvl;
  ddata->subintvl = xi;

  return feat;
}

void interp_step( IplImage*** dog_pyr, int octv, int intvl, int r, int c, double* xi, double* xr, double* xc )
{
  CvMat* dD, * H, * H_inv, X;
  double x[3] = { 0 };
  
  dD = deriv_3D( dog_pyr, octv, intvl, r, c );
  H = hessian_3D( dog_pyr, octv, intvl, r, c );
  H_inv = cvCreateMat( 3, 3, CV_64FC1 );
  cvInvert( H, H_inv, CV_SVD );
  cvInitMatHeader( &X, 3, 1, CV_64FC1, x, CV_AUTOSTEP );
  cvGEMM( H_inv, dD, -1, NULL, 0, &X, 0 );
  
  cvReleaseMat( &dD );
  cvReleaseMat( &H );
  cvReleaseMat( &H_inv );

  *xi = x[2];
  *xr = x[1];
  *xc = x[0];
}

CvMat* deriv_3D( IplImage*** dog_pyr, int octv, int intvl, int r, int c )
{
  CvMat* dI;
  double dx, dy, ds;

  dx = ( pixval32f( dog_pyr[octv][intvl], r, c+1 ) -
	 pixval32f( dog_pyr[octv][intvl], r, c-1 ) ) / 2.0;
  dy = ( pixval32f( dog_pyr[octv][intvl], r+1, c ) -
	 pixval32f( dog_pyr[octv][intvl], r-1, c ) ) / 2.0;
  ds = ( pixval32f( dog_pyr[octv][intvl+1], r, c ) -
	 pixval32f( dog_pyr[octv][intvl-1], r, c ) ) / 2.0;
  
  dI = cvCreateMat( 3, 1, CV_64FC1 );
  cvmSet( dI, 0, 0, dx );
  cvmSet( dI, 1, 0, dy );
  cvmSet( dI, 2, 0, ds );

  return dI;
}

CvMat* hessian_3D( IplImage*** dog_pyr, int octv, int intvl, int r, int c )
{
  CvMat* H;
  double v, dxx, dyy, dss, dxy, dxs, dys;
  
  v = pixval32f( dog_pyr[octv][intvl], r, c );
  dxx = ( pixval32f( dog_pyr[octv][intvl], r, c+1 ) + 
	  pixval32f( dog_pyr[octv][intvl], r, c-1 ) - 2 * v );
  dyy = ( pixval32f( dog_pyr[octv][intvl], r+1, c ) +
	  pixval32f( dog_pyr[octv][intvl], r-1, c ) - 2 * v );
  dss = ( pixval32f( dog_pyr[octv][intvl+1], r, c ) +
	  pixval32f( dog_pyr[octv][intvl-1], r, c ) - 2 * v );
  dxy = ( pixval32f( dog_pyr[octv][intvl], r+1, c+1 ) -
	  pixval32f( dog_pyr[octv][intvl], r+1, c-1 ) -
	  pixval32f( dog_pyr[octv][intvl], r-1, c+1 ) +
	  pixval32f( dog_pyr[octv][intvl], r-1, c-1 ) ) / 4.0;
  dxs = ( pixval32f( dog_pyr[octv][intvl+1], r, c+1 ) -
	  pixval32f( dog_pyr[octv][intvl+1], r, c-1 ) -
	  pixval32f( dog_pyr[octv][intvl-1], r, c+1 ) +
	  pixval32f( dog_pyr[octv][intvl-1], r, c-1 ) ) / 4.0;
  dys = ( pixval32f( dog_pyr[octv][intvl+1], r+1, c ) -
	  pixval32f( dog_pyr[octv][intvl+1], r-1, c ) -
	  pixval32f( dog_pyr[octv][intvl-1], r+1, c ) +
	  pixval32f( dog_pyr[octv][intvl-1], r-1, c ) ) / 4.0;
  
  H = cvCreateMat( 3, 3, CV_64FC1 );
  cvmSet( H, 0, 0, dxx );
  cvmSet( H, 0, 1, dxy );
  cvmSet( H, 0, 2, dxs );
  cvmSet( H, 1, 0, dxy );
  cvmSet( H, 1, 1, dyy );
  cvmSet( H, 1, 2, dys );
  cvmSet( H, 2, 0, dxs );
  cvmSet( H, 2, 1, dys );
  cvmSet( H, 2, 2, dss );

  return H;
}

double interp_contr( IplImage*** dog_pyr, int octv, int intvl, int r, int c, double xi, double xr, double xc )
{
  CvMat* dD, X, T;
  double t;
  double x[3] = { xc, xr, xi };

  cvInitMatHeader( &X, 3, 1, CV_64FC1, x, CV_AUTOSTEP );
  cvInitMatHeader( &T, 1, 1, CV_64FC1, &t, CV_AUTOSTEP );
  dD = deriv_3D( dog_pyr, octv, intvl, r, c );
  cvGEMM( dD, &X, 1, NULL, 0, &T,  CV_GEMM_A_T );
  cvReleaseMat( &dD );
  
  return pixval32f( dog_pyr[octv][intvl], r, c ) + t / 2;
}

struct feature* new_feature( void )
{
  struct feature* feat;
  struct detection_data* ddata;

  feat = ( feature* )malloc( sizeof( struct feature ) );
  memset( feat, 0, sizeof( struct feature ) );
  ddata = ( detection_data* )malloc( sizeof( struct detection_data ) );
  memset( ddata, 0, sizeof( struct detection_data ) );
  feat->feature_data = ddata;
  feat->type = FEATURE_LOWE;

  return feat;
}

int is_too_edge_like( IplImage* dog_img, int r, int c, int curv_thr )
{
  double dxx = pixval32f( dog_img, r, c+1 ) + pixval32f( dog_img, r, c-1 ) - 2 * pixval32f(dog_img, r, c);
  double dyy = pixval32f( dog_img, r+1, c ) + pixval32f( dog_img, r-1, c ) - 2 * pixval32f(dog_img, r, c);
  double dxy = ( pixval32f(dog_img, r+1, c+1) - pixval32f(dog_img, r+1, c-1) - pixval32f(dog_img, r-1, c+1) + pixval32f(dog_img, r-1, c-1) ) / 4.0;
  double tr = dxx + dyy;
  double det = dxx * dyy - dxy * dxy;

  if( det <= 0 ) return 1;
  if( tr * tr / det < ( curv_thr + 1.0 )*( curv_thr + 1.0 ) / curv_thr ) return 0;
  return 1;
}

void calc_feature_scales( CvSeq* features, double sigma, int intvls )
{
  struct feature* feat;
  struct detection_data* ddata;
  double intvl;

  int n = features->total;
  for( int i = 0; i < n; i++ ) {
      feat = CV_GET_SEQ_ELEM( struct feature, features, i );
      ddata = feat_detection_data( feat );
      intvl = ddata->intvl + ddata->subintvl;
      feat->scl = sigma * pow( 2.0, ddata->octv + intvl / intvls );
      ddata->scl_octv = sigma * pow( 2.0, intvl / intvls );
    }
}

void adjust_for_img_dbl( CvSeq* features )
{
  struct feature* feat;
  
  int n = features->total;
  for( int i = 0; i < n; i++ ) {
    feat = CV_GET_SEQ_ELEM( struct feature, features, i );
    feat->x /= 2.0;
    feat->y /= 2.0;
    feat->scl /= 2.0;
    feat->img_pt.x /= 2.0;
    feat->img_pt.y /= 2.0;
  }
}

void calc_feature_oris( CvSeq* features, IplImage*** gauss_pyr )
{
  struct feature* feat;
  struct detection_data* ddata;
  double* hist;
  double omax;
  int n = features->total;

  for( int i = 0; i < n; i++ ) {
    feat = ( feature* )malloc( sizeof( struct feature ) );
    cvSeqPopFront( features, feat );
    ddata = feat_detection_data( feat );
    hist = ori_hist( gauss_pyr[ddata->octv][ddata->intvl], ddata->r, ddata->c, SIFT_ORI_HIST_BINS, cvRound( SIFT_ORI_RADIUS * ddata->scl_octv ), SIFT_ORI_SIG_FCTR * ddata->scl_octv );
    for( int j = 0; j < SIFT_ORI_SMOOTH_PASSES; j++ )
  	smooth_ori_hist( hist, SIFT_ORI_HIST_BINS );
    omax = dominant_ori( hist, SIFT_ORI_HIST_BINS );
    add_good_ori_features( features, hist, SIFT_ORI_HIST_BINS,
			    omax * SIFT_ORI_PEAK_RATIO, feat );
    free( ddata );
    free( feat );
    free( hist );
  }
}

double* ori_hist( IplImage* img, int r, int c, int n, int rad, double sigma )
{
  double* hist;
  double mag, ori, w, exp_denom, PI2 = CV_PI * 2.0;
  int bin, i, j;

  hist = ( double* )calloc( n, sizeof( double ) );
  exp_denom = 2.0 * sigma * sigma;
  for( i = -rad; i <= rad; i++ )
    for( j = -rad; j <= rad; j++ )
      if( calc_grad_mag_ori( img, r + i, c + j, &mag, &ori ) )
	{
	  w = exp( -( i*i + j*j ) / exp_denom );
	  bin = cvRound( n * ( ori + CV_PI ) / PI2 );
	  bin = ( bin < n )? bin : 0;
	  hist[bin] += w * mag;
	}

  return hist;
}

int calc_grad_mag_ori( IplImage* img, int r, int c, double* mag, double* ori )
{
  double dx, dy;

  if( r > 0  &&  r < img->height - 1  &&  c > 0  &&  c < img->width - 1 )
    {
      dx = pixval32f( img, r, c+1 ) - pixval32f( img, r, c-1 );
      dy = pixval32f( img, r-1, c ) - pixval32f( img, r+1, c );
      *mag = sqrt( dx*dx + dy*dy );
      *ori = atan2( dy, dx );
      return 1;
    }

  else return 0;
}

void smooth_ori_hist( double* hist, int n )
{
  double prev, tmp, h0 = hist[0];
  int i;

  prev = hist[n-1];
  for( i = 0; i < n; i++ )
    {
      tmp = hist[i];
      hist[i] = 0.25 * prev + 0.5 * hist[i] + 
	0.25 * ( ( i+1 == n )? h0 : hist[i+1] );
      prev = tmp;
    }
}

double dominant_ori( double* hist, int n )
{
  double omax;
  int maxbin, i;

  omax = hist[0];
  maxbin = 0;
  for( i = 1; i < n; i++ )
    if( hist[i] > omax )
      {
	omax = hist[i];
	maxbin = i;
      }
  return omax;
}

#define interp_hist_peak( l, c, r ) ( 0.5 * ((l)-(r)) / ((l) - 2.0*(c) + (r)) )

void add_good_ori_features( CvSeq* features, double* hist, int n, double mag_thr, struct feature* feat )
{
  struct feature* new_feat;
  double bin, PI2 = CV_PI * 2.0;
  int l, r, i;

  for( i = 0; i < n; i++ )
    {
      l = ( i == 0 )? n - 1 : i-1;
      r = ( i + 1 ) % n;
      
      if( hist[i] > hist[l]  &&  hist[i] > hist[r]  &&  hist[i] >= mag_thr )
	{
	  bin = i + interp_hist_peak( hist[l], hist[i], hist[r] );
	  bin = ( bin < 0 )? n + bin : ( bin >= n )? bin - n : bin;
	  new_feat = clone_feature( feat );
	  new_feat->ori = ( ( PI2 * bin ) / n ) - CV_PI;
	  cvSeqPush( features, new_feat );
	  free( new_feat );
	}
    }
}

struct feature* clone_feature( struct feature* feat )
{
  struct feature* new_feat;
  struct detection_data* ddata;

  new_feat = new_feature();
  ddata = feat_detection_data( new_feat );
  memcpy( new_feat, feat, sizeof( struct feature ) );
  memcpy( ddata, feat_detection_data(feat), sizeof( struct detection_data ) );
  new_feat->feature_data = ddata;

  return new_feat;
}

void compute_descriptors( CvSeq* features, IplImage*** gauss_pyr, int d, int n )
{
  struct feature* feat;
  struct detection_data* ddata;
  double*** hist;
  int i, k = features->total;

  for( i = 0; i < k; i++ )
    {
      feat = CV_GET_SEQ_ELEM( struct feature, features, i );
      ddata = feat_detection_data( feat );
      hist = descr_hist( gauss_pyr[ddata->octv][ddata->intvl], ddata->r,
			 ddata->c, feat->ori, ddata->scl_octv, d, n );
      hist_to_descr( hist, d, n, feat );
      release_descr_hist( &hist, d );
    }
}

double*** descr_hist( IplImage* img, int r, int c, double ori, double scl, int d, int n )
{
  double*** hist;
  double cos_t, sin_t, hist_width, exp_denom, r_rot, c_rot, grad_mag,
    grad_ori, w, rbin, cbin, obin, bins_per_rad, PI2 = 2.0 * CV_PI;
  int radius, i, j;

  hist = ( double*** )calloc( d, sizeof( double** ) );
  for( i = 0; i < d; i++ )
    {
      hist[i] = ( double** )calloc( d, sizeof( double* ) );
      for( j = 0; j < d; j++ )
	hist[i][j] = ( double* )calloc( n, sizeof( double ) );
    }
  
  cos_t = cos( ori );
  sin_t = sin( ori );
  bins_per_rad = n / PI2;
  exp_denom = d * d * 0.5;
  hist_width = SIFT_DESCR_SCL_FCTR * scl;
  radius = hist_width * sqrt((float)2) * ( d + 1.0 ) * 0.5 + 0.5;
  for( i = -radius; i <= radius; i++ )
    for( j = -radius; j <= radius; j++ )
      {
	c_rot = ( j * cos_t - i * sin_t ) / hist_width;
	r_rot = ( j * sin_t + i * cos_t ) / hist_width;
	rbin = r_rot + d / 2 - 0.5;
	cbin = c_rot + d / 2 - 0.5;
	
	if( rbin > -1.0  &&  rbin < d  &&  cbin > -1.0  &&  cbin < d )
	  if( calc_grad_mag_ori( img, r + i, c + j, &grad_mag, &grad_ori ))
	    {
	      grad_ori -= ori;
	      while( grad_ori < 0.0 )
		grad_ori += PI2;
	      while( grad_ori >= PI2 )
		grad_ori -= PI2;
	      
	      obin = grad_ori * bins_per_rad;
	      w = exp( -(c_rot * c_rot + r_rot * r_rot) / exp_denom );
	      interp_hist_entry( hist, rbin, cbin, obin, grad_mag * w, d, n );
	    }
      }

  return hist;
}

void interp_hist_entry( double*** hist, double rbin, double cbin, double obin, double mag, int d, int n )
{
  double d_r, d_c, d_o, v_r, v_c, v_o;
  double** row, * h;
  int r0, c0, o0, rb, cb, ob, r, c, o;

  r0 = cvFloor( rbin );
  c0 = cvFloor( cbin );
  o0 = cvFloor( obin );
  d_r = rbin - r0;
  d_c = cbin - c0;
  d_o = obin - o0;

  for( r = 0; r <= 1; r++ )
    {
      rb = r0 + r;
      if( rb >= 0  &&  rb < d )
	{
	  v_r = mag * ( ( r == 0 )? 1.0 - d_r : d_r );
	  row = hist[rb];
	  for( c = 0; c <= 1; c++ )
	    {
	      cb = c0 + c;
	      if( cb >= 0  &&  cb < d )
		{
		  v_c = v_r * ( ( c == 0 )? 1.0 - d_c : d_c );
		  h = row[cb];
		  for( o = 0; o <= 1; o++ )
		    {
		      ob = ( o0 + o ) % n;
		      v_o = v_c * ( ( o == 0 )? 1.0 - d_o : d_o );
		      h[ob] += v_o;
		    }
		}
	    }
	}
    }
}

void hist_to_descr( double*** hist, int d, int n, struct feature* feat )
{
  int int_val, i, r, c, o, k = 0;

  for( r = 0; r < d; r++ )
    for( c = 0; c < d; c++ )
      for( o = 0; o < n; o++ )
	feat->descr[k++] = hist[r][c][o];

  feat->d = k;
  normalize_descr( feat );
  for( i = 0; i < k; i++ )
    if( feat->descr[i] > SIFT_DESCR_MAG_THR )
      feat->descr[i] = SIFT_DESCR_MAG_THR;
  normalize_descr( feat );

  for( i = 0; i < k; i++ )
    {
      int_val = SIFT_INT_DESCR_FCTR * feat->descr[i];
      feat->descr[i] = MIN( 255, int_val );
    }
}

void normalize_descr( struct feature* feat )
{
  double cur, len_inv, len_sq = 0.0;
  int i, d = feat->d;

  for( i = 0; i < d; i++ )
    {
      cur = feat->descr[i];
      len_sq += cur*cur;
    }
  len_inv = 1.0 / sqrt( len_sq );
  for( i = 0; i < d; i++ )
    feat->descr[i] *= len_inv;
}

int feature_cmp( void* feat1, void* feat2, void* param )
{
  struct feature* f1 = (struct feature*) feat1;
  struct feature* f2 = (struct feature*) feat2;

  if( f1->scl < f2->scl )
    return 1;
  if( f1->scl > f2->scl )
    return -1;
  return 0;
}

void release_descr_hist( double**** hist, int d )
{
  int i, j;

  for( i = 0; i < d; i++)
    {
      for( j = 0; j < d; j++ )
	free( (*hist)[i][j] );
      free( (*hist)[i] );
    }
  free( *hist );
  *hist = NULL;
}

void release_pyr( IplImage**** pyr, int octvs, int n )
{
  int i, j;
  for( i = 0; i < octvs; i++ )
    {
      for( j = 0; j < n; j++ )
	cvReleaseImage( &(*pyr)[i][j] );
      free( (*pyr)[i] );
    }
  free( *pyr );
  *pyr = NULL;
}

inline double distance_2D( CvPoint2D64f pt1, CvPoint2D64f pt2)
{
	return sqrt( double((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y)) );
}

CvMat* transform( CvPoint2D64f* pts, CvPoint2D64f* mpts, int n )
{
	CvMat* H, * A, * VT, * D, h, v9;
	double _h[9];
	int i;

	if( n < 4 )
		return NULL;
	
	A = cvCreateMat( 2*n, 9, CV_64FC1 );
	cvZero( A );
	for( i = 0; i < n; i++ )
    {
        cvmSet( A, 2*i, 3, -pts[i].x );
		cvmSet( A, 2*i, 4, -pts[i].y );
		cvmSet( A, 2*i, 5, -1.0  );
		cvmSet( A, 2*i, 6, mpts[i].y * pts[i].x );
		cvmSet( A, 2*i, 7, mpts[i].y * pts[i].y );
		cvmSet( A, 2*i, 8, mpts[i].y );
		cvmSet( A, 2*i+1, 0, pts[i].x );
		cvmSet( A, 2*i+1, 1, pts[i].y );
		cvmSet( A, 2*i+1, 2, 1.0  );
		cvmSet( A, 2*i+1, 6, -mpts[i].x * pts[i].x );
		cvmSet( A, 2*i+1, 7, -mpts[i].x * pts[i].y );
		cvmSet( A, 2*i+1, 8, -mpts[i].x );
    }
	D = cvCreateMat( 9, 9, CV_64FC1 );
	VT = cvCreateMat( 9, 9, CV_64FC1 );
	cvSVD( A, D, NULL, VT, CV_SVD_MODIFY_A + CV_SVD_V_T );
	v9 = cvMat( 1, 9, CV_64FC1, NULL );
	cvGetRow( VT, &v9, 8 );
	h = cvMat( 1, 9, CV_64FC1, _h );
	cvCopy( &v9, &h, NULL );
	h = cvMat( 3, 3, CV_64FC1, _h );
	H = cvCreateMat( 3, 3, CV_64FC1 );
	cvConvert( &h, H );

	cvReleaseMat( &A );
    cvReleaseMat( &D );
	cvReleaseMat( &VT );
	return H;
}

CvMat* lsq_homog( CvPoint2D64f* pts, CvPoint2D64f* mpts, int n )
{
  CvMat* H, * A, * B, X;
  double x[9];
  int i;

  if( n < 4 )
      return NULL;

  A = cvCreateMat( 2*n, 8, CV_64FC1 );
  B = cvCreateMat( 2*n, 1, CV_64FC1 );
  X = cvMat( 8, 1, CV_64FC1, x );
  H = cvCreateMat(3, 3, CV_64FC1);
  cvZero( A );
  for( i = 0; i < n; i++ )
    {
      cvmSet( A, i, 0, pts[i].x );
      cvmSet( A, i+n, 3, pts[i].x );
      cvmSet( A, i, 1, pts[i].y );
      cvmSet( A, i+n, 4, pts[i].y );
      cvmSet( A, i, 2, 1.0 );
      cvmSet( A, i+n, 5, 1.0 );
      cvmSet( A, i, 6, -pts[i].x * mpts[i].x );
      cvmSet( A, i, 7, -pts[i].y * mpts[i].x );
      cvmSet( A, i+n, 6, -pts[i].x * mpts[i].y );
      cvmSet( A, i+n, 7, -pts[i].y * mpts[i].y );
      cvmSet( B, i, 0, mpts[i].x );
      cvmSet( B, i+n, 0, mpts[i].y );
    }
  cvSolve( A, B, &X, CV_SVD );
  x[8] = 1.0;
  X = cvMat( 3, 3, CV_64FC1, x );
  cvConvert( &X, H );

  cvReleaseMat( &A );
  cvReleaseMat( &B );
  return H;
}

CvPoint2D64f persp_xform_pt( CvPoint2D64f pt, CvMat* T )
{
  CvMat XY, UV;
  double xy[3] = { pt.x, pt.y, 1.0 }, uv[3] = { 0 };
  CvPoint2D64f rslt;

  cvInitMatHeader( &XY, 3, 1, CV_64FC1, xy, CV_AUTOSTEP );
  cvInitMatHeader( &UV, 3, 1, CV_64FC1, uv, CV_AUTOSTEP );
  cvMatMul( T, &XY, &UV );
  rslt = cvPoint2D64f( uv[0] / uv[2], uv[1] / uv[2] );

  return rslt;
}

double error( CvPoint2D64f pt, CvPoint2D64f mpt, CvMat* H )
{
  CvPoint2D64f xpt = persp_xform_pt( pt, H );
  
  return sqrt( distance_2D( xpt, mpt ) );
}

inline struct feature* get_match( struct feature* feat, int mtype )
{
  if( mtype == FEATURE_MDL_MATCH )
    return feat->mdl_match;
  if( mtype == FEATURE_BCK_MATCH )
    return feat->bck_match;
  if( mtype == FEATURE_FWD_MATCH )
    return feat->fwd_match;
  return NULL;
}

int get_matched_features( struct feature* features, int n, int mtype, struct feature*** matched )
{
  struct feature** _matched;
  struct ransac_data* rdata;
  int i, m = 0;

  _matched = (feature**)calloc( n, sizeof( struct feature* ) );
  for( i = 0; i < n; i++ )
  {
	  if( get_match( features + i, mtype ) )
      {
	      rdata = (ransac_data*)malloc( sizeof( struct ransac_data ) );
		  memset( rdata, 0, sizeof( struct ransac_data ) );
		  rdata->orig_feat_data = features[i].feature_data;
		  _matched[m] = features + i;
		  _matched[m]->feature_data = rdata;
		m++;
      }
  }
  *matched = _matched;
  return m;
}

inline double log_factorial( int n )
{
  double f = 0;
  for( int i = 1; i <= n; i++ )
    f += log(double(i));

  return f;
}

int calc_min_inliers( int n, int m, double p_badsupp, double p_badxform )
{
  double pi, sum;
  int i, j;

  for( j = m+1; j <= n; j++ )
    {
      sum = 0;
      for( i = j; i <= n; i++ )
	{
	  pi = (i-m) * log( p_badsupp ) + (n-i+m) * log( 1.0 - p_badsupp ) +
	    log_factorial( n - m ) - log_factorial( i - m ) -
	    log_factorial( n - i );
	  sum += exp( pi );
	}
      if( sum < p_badxform )
	break;
    }
  return j;
}

struct feature** draw_ransac_sample( struct feature** features, int n, int m )
{
  struct feature** sample, * feat;
  struct ransac_data* rdata;
  int i, x;

  for( i = 0; i < n; i++ )
  {
	  rdata = feat_ransac_data( features[i] );
      rdata->sampled = 0;
  }

  sample = (feature**)calloc( m, sizeof( struct feature* ) );
  for( i = 0; i < m; i++ )
  {
	  do
	  {
		  x = rand() % n;
		  feat = features[x];
		  rdata = feat_ransac_data( feat );
	  }
	  while( rdata->sampled );
      sample[i] = feat;
      rdata->sampled = 1;
  }
  return sample;
}

void extract_corresp_pts( struct feature** features, int n, int mtype, CvPoint2D64f** pts, CvPoint2D64f** mpts )
{
  struct feature* match;
  CvPoint2D64f* _pts, * _mpts;
  int i;

  _pts = (CvPoint2D64f*)calloc( n, sizeof( CvPoint2D64f ) );
  _mpts = (CvPoint2D64f*)calloc( n, sizeof( CvPoint2D64f ) );

  if( mtype == FEATURE_MDL_MATCH )
    for( i = 0; i < n; i++ )
    {
		match = get_match( features[i], mtype );
		_pts[i] = features[i]->img_pt;
		_mpts[i] = match->mdl_pt;
    }

  else
    for( i = 0; i < n; i++ )
    {
		match = get_match( features[i], mtype );
		_pts[i] = features[i]->img_pt;
		_mpts[i] = match->img_pt;
    }

  *pts = _pts;
  *mpts = _mpts;
}

int find_consensus( struct feature** features, int n, int mtype, CvMat* M, ransac_err_fn err_fn, double err_tol, struct feature*** consensus )
{
  struct feature** _consensus;
  struct feature* match;
  CvPoint2D64f pt, mpt;
  double err;
  int i, in = 0;

  _consensus = (feature**)calloc( n, sizeof( struct feature* ) );

  if( mtype == FEATURE_MDL_MATCH )
    for( i = 0; i < n; i++ )
    {
		match = get_match( features[i], mtype );
		pt = features[i]->img_pt;
		mpt = match->mdl_pt;
		err = err_fn( pt, mpt, M );
		if( err <= err_tol )
			_consensus[in++] = features[i];
    }

  else
    for( i = 0; i < n; i++ )
	{
		match = get_match( features[i], mtype );
		pt = features[i]->img_pt;
		mpt = match->img_pt;
		err = err_fn( pt, mpt, M );
		if( err <= err_tol )
			_consensus[in++] = features[i];
     }
  *consensus = _consensus;
  return in;
}

inline void release_mem( CvPoint2D64f* pts1, CvPoint2D64f* pts2, struct feature** features )
{
  free( pts1 );
  free( pts2 );
  if( features )
    free( features );
}

CvMat* ransac_xform( struct feature* features, int n, int mtype,
		     ransac_xform_fn xform_fn, int m, double p_badxform,
		     ransac_err_fn err_fn, double err_tol,
		     struct feature*** inliers, int* n_in )
{
  struct feature** matched, ** sample, ** consensus, ** consensus_max = NULL;
  struct ransac_data* rdata;
  CvPoint2D64f* pts, * mpts;
  CvMat* M = NULL;
  double p, in_frac = RANSAC_INLIER_FRAC_EST;
  int i, nm, in, in_min, in_max = 0, k = 0;

  nm = get_matched_features( features, n, mtype, &matched );
  if( nm < m )
      goto end;

  srand( time(NULL) );

  in_min = calc_min_inliers( nm, m, RANSAC_PROB_BAD_SUPP, p_badxform );
  p = pow( 1.0 - pow( in_frac, m ), k );
  while( p > p_badxform )
  {
      sample = draw_ransac_sample( matched, nm, m );
      extract_corresp_pts( sample, m, mtype, &pts, &mpts );
      M = xform_fn( pts, mpts, m );
      if( ! M )
		goto iteration_end;
      in = find_consensus( matched, nm, mtype, M, err_fn, err_tol, &consensus);
      if( in > in_max )
	  {
		if( consensus_max )
			free( consensus_max );
		consensus_max = consensus;
		in_max = in;
		in_frac = (double)in_max / nm;
	  }
      else
		free( consensus );
      cvReleaseMat( &M );

	  iteration_end:
		release_mem( pts, mpts, sample );
		p = pow( 1.0 - pow( in_frac, m ), ++k );
  }

  if( in_max >= in_min )
  {
      extract_corresp_pts( consensus_max, in_max, mtype, &pts, &mpts );
      M = xform_fn( pts, mpts, in_max );
      in = find_consensus( matched, nm, mtype, M, err_fn, err_tol, &consensus);
      cvReleaseMat( &M );
      release_mem( pts, mpts, consensus_max );
      extract_corresp_pts( consensus, in, mtype, &pts, &mpts );
      M = xform_fn( pts, mpts, in );
      if( inliers )
	  {
		*inliers = consensus;
		consensus = NULL;
	  }
      if( n_in )
		*n_in = in;
      release_mem( pts, mpts, consensus );
  }
  else if( consensus_max )
  {
    if( inliers )
		*inliers = NULL;
    if( n_in )
		*n_in = 0;
    free( consensus_max );
  }

  end:
	for( i = 0; i < nm; i++ )
    {
      rdata = feat_ransac_data( matched[i] );
      matched[i]->feature_data = rdata->orig_feat_data;
      free( rdata );
    }
	free( matched );
	return M;
}
#endif
