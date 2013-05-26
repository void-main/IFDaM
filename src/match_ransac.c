#include "sift.h"
#include "imgfeatures.h"
#include "kdtree.h"
#include "utils.h"
#include "xform.h"

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include <stdio.h>

/* the maximum number of keypoint NN candidates to check during BBF search */
#define KDTREE_BBF_MAX_NN_CHKS 200

/* threshold on squared ratio of distances between NN and 2nd NN */
#define NN_SQ_DIST_RATIO_THR 0.6

int main( int argc, char** argv )
{
  IplImage* img1, * img2, * stacked;
  struct feature* feat1, * feat2, * feat;
  struct feature** nbrs;
  struct kd_node* kd_root;
  CvPoint pt1, pt2, pt3, pt4;
  double d0, d1;
  int n1, n2, k, i, m = 0;

  if( argc != 3 )
    fatal_error( "usage: %s <img1> <img2>", argv[0] );

  img1 = cvLoadImage( argv[1], 1 );
  if( ! img1 )
    fatal_error( "unable to load image from %s", argv[1] );
  img2 = cvLoadImage( argv[2], 1 );
  if( ! img2 )
    fatal_error( "unable to load image from %s", argv[2] );
  stacked = stack_imgs( img1, img2 );

  fprintf( stderr, "Finding features in %s...\n", argv[1] );
  n1 = sift_features( img1, &feat1 );
  fprintf( stderr, "Finding features in %s...\n", argv[2] );
  n2 = sift_features( img2, &feat2 );
  fprintf( stderr, "Building kd tree...\n" );
  kd_root = kdtree_build( feat2, n2 );
  for( i = 0; i < n1; i++ )
  {
    feat = feat1 + i;
    k = kdtree_bbf_knn( kd_root, feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS );
    if( k == 2 )
    {
      d0 = descr_dist_sq( feat, nbrs[0] );
      d1 = descr_dist_sq( feat, nbrs[1] );
      if( d0 < d1 * NN_SQ_DIST_RATIO_THR )
      {
        m++;
        feat1[i].fwd_match = nbrs[0];
      }
    }
    free( nbrs );
  }

  CvMat* H;
  int nr;
  struct feature** inliner;
  H = ransac_xform( feat1, n1, FEATURE_FWD_MATCH, lsq_homog, 4, 0.01,
   homog_xfer_err, 3.0, &inliner, &nr);
  if( H )
  {
    cvReleaseMat( &H );

    for( i = 0; i < nr; i++ )
    {
      pt1 = cvPoint( cvRound( inliner[i]->x ), cvRound( inliner[i]->y ) );
      pt2 = cvPoint( cvRound( inliner[i]->fwd_match->x ), cvRound( inliner[i]->fwd_match->y ) );
      pt3 = cvPoint( cvRound( inliner[i]->x ), cvRound( inliner[i]->y + img1->height ) );
      pt4 = cvPoint( cvRound( inliner[i]->fwd_match->x ), cvRound( inliner[i]->fwd_match->y + img1->height ) );
      pt2.x += img1->width;
      cvLine( stacked, pt1, pt2, CV_RGB(0,128,128), 1, 16, 0 );
      draw_x( stacked, pt1, 1, 4, CV_RGB(0, 0, 255));
      draw_x( stacked, pt2, 1, 4, CV_RGB(255, 255, 0));
      cvLine( stacked, pt3, pt4, CV_RGB(255,255,255), 1, 16, 0 );
      draw_x( stacked, pt3, 1, 4, CV_RGB(0, 0, 255));
      draw_x( stacked, pt4, 1, 4, CV_RGB(255, 255, 0));
    }

    fprintf( stderr, "Found %d total matches after ransac\n", nr );
    display_big_img( stacked, "SIFT + RANSAC" );
    cvWaitKey( 0 );

    cvReleaseImage( &stacked );
  }

  cvReleaseImage( &img1 );
  cvReleaseImage( &img2 );
  kdtree_release( kd_root );
  free( feat1 );
  free( feat2 );
  return 0;
}
