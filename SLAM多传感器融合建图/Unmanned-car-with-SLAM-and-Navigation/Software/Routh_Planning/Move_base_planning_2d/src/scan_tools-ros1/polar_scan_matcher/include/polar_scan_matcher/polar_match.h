/***************************************************************************
                          polar_match.h  - matching laser scans in polar coord system
                          designed for use with SICK LMS in cm res./361 meas. mode
                          only 181 readings (1 deg res) are used (less accuracy but higher
                          speed)
                             -------------------
    begin                : Tue Nov 9 2004
    version              : 0.2    
    copyright            : (C) 2005 by Albert Diosi and Lindsay Kleeman
    email                : albert.diosi@gmail.com
    comments             : range units are cm; angle units are deg
                           the laser is on the robots y axis
                           - in scan projections, occluded ref scanpoints aren't removed!
    changed:
                          14/03/2005 removing of unnecessary code
 ***************************************************************************/
 /****************************************************************************
    This file is part of polar_matching.

    polar_matching is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    polar_matching is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with polar_matching; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
****************************************************************************/

#ifndef POLAR_SCAN_MATCHING_POLAR_MATCHER_H
#define POLAR_SCAN_MATCHING_POLAR_MATCHER_H

#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#define PM_TYPE             double // change it to double for higher accuracy  and lower speed

// range reading errors
#define PM_RANGE     1  // range reading is longer than PM_MAX_RANGE
#define PM_MOVING    2  // range reading corresponds to a moving point
#define PM_MIXED     4  // range reading is a mixed pixel
#define PM_OCCLUDED  8  // range reading is occluded
#define PM_EMPTY     16 // at that bearing there is no measurment (between 2
                        //    segments there is no interpolation!)

#define PM_ODO  -1 //show results with odometry only in mapping_with_matching()
#define PM_PSM   1 //polar scanmatching - matching bearing
#define PM_PSM_C 2 //polar scanmatching - using cartesian equations
#define PM_ICP   3 //scanmatchign with iterative closest point
#define PM_IDC   4 //lu's iterative dual correspondence - not implemented,
                   //Guttman's external code is used for comparisson
#define PM_MBICP 5 //scanmatchign with metric based ICP

const double PM_D2R = M_PI/180.0; // degrees to rad
const double PM_R2D = 180.0/M_PI; // rad to degrees

//const int     PM_L_POINTS = 681;

//might want to consider doubles for rx,ry -> in a large environment?
struct PMScan
{
  PMScan(int nPoints)
  {
    r.resize(nPoints);
    x.resize(nPoints);
    y.resize(nPoints);
    bad.resize(nPoints);
    seg.resize(nPoints);
  }

  PM_TYPE  rx;   //robot odometry pos
  PM_TYPE  ry;   //robot odometry pos
  PM_TYPE  th;   //robot orientation 
  std::vector<PM_TYPE>  r;//[cm] - 0 or negative ranges denote invalid readings.
  std::vector<PM_TYPE>  x;//[cm]
  std::vector<PM_TYPE>  y;//[cm]
  std::vector<int>      bad;// 0 if OK
                            //sources of invalidity - too big range;
                            //moving object; occluded;mixed pixel
  std::vector<int>      seg;//nuber describing into which segment the point belongs to
};

class PolarMatcher
{
  private:

    //calculates an error index expressing the quality of the match
    //of the actual scan to the reference scan
    //has to be called after scan matching so the actual scan in expressed
    //in the reference scan coordinate system
    //return the average minimum Euclidean distance; MAXIMUM RANGE points
    //are not considered; number of non maximum range points have to be
    //smaller than a threshold
    PM_TYPE pm_error_index(PMScan *lsr,PMScan *lsa);

    //estimates the covariance matrix(c11,c12,c22,c33) (x,y,th) of
    //a scan match based on an error index (err-depends on how good the
    //match is), and the angle of the corridor if it is a corridor
    void pm_cov_est(PM_TYPE err, double *c11,double *c12, double *c22, double *c33,
                        bool corridor=false, PM_TYPE corr_angle=0);

    void pm_scan_project(const PMScan *act,  PM_TYPE   *new_r,  int *new_bad);
    PM_TYPE pm_orientation_search(const PMScan *ref, const PM_TYPE *new_r, const int *new_bad);
    PM_TYPE pm_translation_estimation(const PMScan *ref, const PM_TYPE *new_r, const int *new_bad, PM_TYPE C, PM_TYPE *dx, PM_TYPE *dy);

    PM_TYPE point_line_distance ( PM_TYPE x1, PM_TYPE y1, PM_TYPE x2, PM_TYPE y2,
                              PM_TYPE x3, PM_TYPE y3,PM_TYPE *x, PM_TYPE *y);

    inline PM_TYPE norm_a ( PM_TYPE a )
    {
      int m;
      m= ( int ) ( a/ ( 2.0*M_PI ) );
      a=a- ( PM_TYPE ) m*M_PI;
      if ( a< ( -M_PI ) )
        a+=2.0*M_PI;
      if ( a>=M_PI )
        a-=2.0*M_PI;
      return ( a );
    }

  public:

    int     PM_L_POINTS;

    PM_TYPE PM_FI_MIN;// = M_PI/2.0 - PM_FOV*PM_D2R/2.0;//[rad] bearing from which laser scans start
    PM_TYPE PM_FI_MAX;// = M_PI/2.0 + PM_FOV*PM_D2R/2.0;//[rad] bearing at which laser scans end
    PM_TYPE PM_DFI;   // = PM_FOV*PM_D2R/ ( PM_L_POINTS + 1.0 );//[rad] angular resolution of laser scans

    std::vector<PM_TYPE>  pm_fi;//contains precomputed angles
    std::vector<PM_TYPE>  pm_si;//contains sinus of angles
    std::vector<PM_TYPE>  pm_co;//contains cos of angles

    double  PM_FOV ;             //! field of view of the laser range finder in degrees
    double  PM_MAX_RANGE ;       //![cm] max valid laser range (set this to about 400 for the Hokuyo URG)
    int     PM_MIN_VALID_POINTS; //! minimum number of valid points for scanmatching
    int     PM_SEARCH_WINDOW;     //! half window size which is searched for correct orientation

    double  PM_TIME_DELAY;       //!<[ms]time delay (time registration error) in the laser measurements

    double  PM_MAX_ERROR;        //[cm] max distance between associated points used in pose est.
    double  PM_STOP_COND;        //! stopping condition; the pose change has to be smaller than ...
    int     PM_MAX_ITER;         //! maximum number of iterations for PSM.PSM_C
    int     PM_MAX_ITER_ICP;     //! maximum number of iterations for ICP,MBICP
    int     PM_STOP_COND_ICP;    //! stopping condition; the pose change has to be smaller than ... for ICP, MBICP

    PolarMatcher();

    void pm_init();

    //filters the ranges with a median filter. x,y points are not upadted
    //ls - laser scan; the job of the median filter is to remove chair and table
    //legs wich would be moving anyway;
    void pm_median_filter(PMScan *ls);

    // marks point further than a given distance PM_MAX_RANGE as PM_RANGE
    void pm_find_far_points(PMScan *ls);

    //segments scanpoints into groups based on range discontinuities
    void pm_segment_scan(PMScan *ls);

    // minimizes least square error through changing lsa->rx, lsa->ry,lsa->th
    // this looks for angle too, like pm_linearized_match_proper,execept it
    // fits a parabola to the error when searching for the angle and interpolates.
    PM_TYPE pm_psm(PMScan *lsr,PMScan *lsa);

    // does scan matching using the equations for translation and orietation
    //estimation as in Lu & milios, however our matching bearing association rule
    //is used together with our association filter.
    //this is PSM-C in the tech report
    PM_TYPE pm_psm_c(PMScan *lsr,PMScan *lsa);
};

#endif //POLAR_SCAN_MATCHING_POLAR_MATCH_H
