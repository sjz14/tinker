#ifndef FEATURES_H
#define FEATURES_H

#include "cxcore.h"

enum feature_type
  {
    FEATURE_OXFD,
    FEATURE_LOWE,
  };

enum feature_match_type
  {
    FEATURE_FWD_MATCH,
    FEATURE_BCK_MATCH,
    FEATURE_MDL_MATCH,
  };

#define FEATURE_MAX_D 128

struct feature
{
  double x;                      
  double y;                      
  double a;                    
  double b;                     
  double c;                    
  double scl;                   
  double ori;                  
  int d;                        
  double descr[FEATURE_MAX_D];   
  int type;                   
  int category;              
  struct feature* fwd_match;  
  struct feature* bck_match;    
  struct feature* mdl_match;    
  CvPoint2D64f img_pt;         
  CvPoint2D64f mdl_pt;        
  void* feature_data;    
};

struct ransac_data
{
  void* orig_feat_data;
  int sampled;
};

#endif
