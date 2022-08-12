/***************************************************************************
             polar_match.cpp  - matching laser scans in polar coord system
             designed for use with SICK LMS in cm res./361 meas. mode
             only 181 readings (1 deg res) are used (less accuracy but higher
             speed) - changes made to accomodate other lasers, but not tested properly yet.
                             -------------------
begin           : Tue Nov 9 2004
version         : 0.2
copyright       : (C) 2005,2006,2007,2008,2009 by Albert Diosi and Lindsay Kleeman
email           : albert.diosi@gmail.com
comments        : - range units are cm; angle units are deg
                  - don't forget to set the optimization switch!
                  - if it works with float->change fabs-fabsf, floor->floorf
                  - why not add angle as well to the iterative least squares?
                     => bad convergence, error tends to be fixed with rotation
                  - try to make the C - coef for sigmoid function smaller with
                     with the as scanmatching converges, to increase the precision
                     as in dudek
changed:
                05/03/2007- add interpolation to icp
                04/11/2005- bug fixed in pm_is_corridor. Bug pointed out by Alan Zhang
                03/08/2005- simple implementation of IDC added not working yet - remove!
                26/05/2005- projection filter of ICP fixed
                7/12/2004 - dx,dy estimation interleaved with dth estimation
                            seems to work OK
                3/12/2004 - iterative range least squares seems to work (for
                           dx,dy only), though it needs to be thoroughly tested

                26/11/2004

edited: (Ivan Dyanovski, ivan.dryanovski@gmail.com)

                11/08/2010 - removed everything related to visualisation
                           - changed defines to private class members
                           - removed methods and variables not needed for basic
                             matching


TODO: change the time measurement approach; do a proper cleanup. have doxygen comments. 
      - Comment colours used when GR is defined.
      - Document all magic constansts.(pm_psm: change of C after 10 steps)               
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

#include "polar_scan_matcher/polar_match.h"

using namespace std;

PolarMatcher::PolarMatcher()
{

}

/** @brief Initialises internar variables
*/
void PolarMatcher::pm_init()
{
  pm_fi.resize(PM_L_POINTS);
  pm_si.resize(PM_L_POINTS);
  pm_co.resize(PM_L_POINTS);

  PM_FI_MIN = M_PI/2.0 - PM_FOV*PM_D2R/2.0; 
  PM_FI_MAX = M_PI/2.0 + PM_FOV*PM_D2R/2.0; 
  PM_DFI    = PM_FOV*PM_D2R/ ( PM_L_POINTS + 1.0 );

  for ( int i=0;i<PM_L_POINTS;i++ )
  {
    pm_fi[i] = ( ( float ) i ) *PM_DFI + PM_FI_MIN;
    pm_si[i] = sin ( pm_fi[i] );
    pm_co[i] = cos ( pm_fi[i] );
  }
}//pm_init

//-------------------------------------------------------------------------
//filters the ranges with a median filter. x,y points are not upadted
//ls - laser scan
// seems like the median filter is a good thing!
//if window is 5, then 3 points are needed in a bunch to surrive!
//don't use this function with line fitting!
void PolarMatcher::pm_median_filter ( PMScan *ls )
{
  const int HALF_WINDOW  = 2;//2 to left 2 to right
  const int WINDOW = 2*HALF_WINDOW+1;
  PM_TYPE   r[WINDOW];
  PM_TYPE   w;

  int i,j,k,l;

  for ( i=0;i<PM_L_POINTS;i++ )
  {
    k=0;
    for ( j=i-HALF_WINDOW;j<=i+HALF_WINDOW;j++ )
    {
      l = ( ( j>=0 ) ?j:0 );
      r[k]=ls->r[ ( ( l < PM_L_POINTS ) ?l: ( PM_L_POINTS-1 ) ) ];
      k++;
    }
    //bubble sort r
    for ( j= ( WINDOW-1 );j>0;j-- )
      for ( k=0;k<j;k++ )
        if ( r[k]>r[k+1] ) // wrong order? - swap them
        {
          w=r[k];
          r[k]=r[k+1];
          r[k+1] = w;
        }
    ls->r[i] = r[HALF_WINDOW];//choose the middle point
  }
}

//-------------------------------------------------------------------------
// segments scanpoints into groups based on range discontinuities
// number 0 is reserved to segments with size 1
// assumptions: too far points PM_MAX_RANGE - divide segments
//              - bad points are only due to far points and mixed pixels
// seems all right, except a far point can be the beginning of a new segment
// if the next point is good and close -> it shouldn't make a difference
void PolarMatcher::pm_segment_scan ( PMScan *ls )
{
  const PM_TYPE   MAX_DIST = 20.0;//max range diff between conseq. points in a seg
  PM_TYPE   dr;
  int       seg_cnt = 0;
  int       i,cnt;
  bool      break_seg;

  seg_cnt = 1;

  //init:
  if ( fabs ( ls->r[0]-ls->r[1] ) <MAX_DIST ) //are they in the same segment?
  {
    ls->seg[0] = seg_cnt;
    ls->seg[1] = seg_cnt;
    cnt        = 2;    //2 points in the segment
  }
  else
  {
    ls->seg[0] = 0; //point is a segment in itself
    ls->seg[1] = seg_cnt;
    cnt        = 1;
  }

  for ( i=2;i<PM_L_POINTS;i++ )
  {
    //segment breaking conditions: - bad point;
    break_seg = false;
    if ( ls->bad[i] )
    {
      break_seg = true;
      ls->seg[i] = 0;
    }
    else
    {
      dr = ls->r[i]- ( 2.0*ls->r[i-1]-ls->r[i-2] );//extrapolate & calc difference
      if ( fabs ( ls->r[i]-ls->r[i-1] ) <MAX_DIST || ( ( ls->seg[i-1]==ls->seg[i-2] )
              && fabs ( dr ) <MAX_DIST ) )
      {
        //not breaking the segment
        cnt++;
        ls->seg[i] = seg_cnt;
      }
      else
        break_seg = true;
    }//if ls
    if ( break_seg ) // breaking the segment?
    {
      if ( cnt==1 )
      {
        //check first if the last three are not on a line by coincidence
        dr = ls->r[i]- ( 2.0*ls->r[i-1]-ls->r[i-2] );
        if ( ls->seg[i-2] == 0 && ls->bad[i] == 0 && ls->bad[i-1] == 0
                && ls->bad[i-2] == 0 && fabs ( dr ) <MAX_DIST )
        {
          ls->seg[i]   = seg_cnt;
          ls->seg[i-1] = seg_cnt;
          ls->seg[i-2] = seg_cnt;
          cnt = 3;
        }//if ls->
        else
        {
          ls->seg[i-1] = 0;
          //what if ls[i] is a bad point? - it could be the start of a new
          //segment if the next point is a good point and is close enough!
          //in that case it doesn't really matters
          ls->seg[i] = seg_cnt;//the current point is a new segment
          cnt = 1;
        }
      }//if cnt ==1
      else
      {
        seg_cnt++;
        ls->seg[i] = seg_cnt;
        cnt = 1;
      }//else if cnt
    }//if break seg
  }//for
}//pm_segment_scan

//-------------------------------------------------------------------------
// marks point further than a given distance PM_MAX_RANGE as PM_RANGE
void PolarMatcher::pm_find_far_points ( PMScan *ls )
{
  for ( int i=0;i<PM_L_POINTS;i++ )
  {
    if ( ls->r[i]>PM_MAX_RANGE )
      ls->bad[i] |= PM_RANGE;
  }//
}

//-------------------------------------------------------------------------
//calculates an error index expressing the quality of the match
//of the actual scan to the reference scan
//has to be called after scan matching so the actual scan in expressed
//in the reference scan coordinate system
//return the average minimum Euclidean distance; MAXIMUM RANGE points
//are not considered; number of non maximum range points have to be
//smaller than a threshold
//actual scan is compared to reference scan and vice versa, maximum is
//taken
//I could implement it in polar frame as well, would be O(n)
PM_TYPE PolarMatcher::pm_error_index ( PMScan *lsr,PMScan *lsa )
{
  int     i,j;
  PM_TYPE rx[PM_L_POINTS],ry[PM_L_POINTS],ax[PM_L_POINTS],ay[PM_L_POINTS];
  PM_TYPE x,y;
  PM_TYPE d,dmin,dsum,dx,dy;
  PM_TYPE dsum1;
  int     n,n1,rn=0,an=0;
  const   PM_TYPE HUGE_ERROR       = 1000000;
  const   int     MIN_POINTS = 100;

  lsa->th = norm_a ( lsa->th );
  PM_TYPE co = cos ( lsa->th ),si = sin ( lsa->th );
  PM_TYPE c,sig;

  //x axis equation si*x-co*y+c=0
  c = - ( lsa->rx*si-lsa->ry*co );//calc for the x axis of the actual frame
  //"signum" of a point from the lasers view substituted into the equation
  sig = si* ( lsa->rx+cos ( lsa->th+0.1 ) )-co* ( lsa->ry+sin ( lsa->th+0.1 ) ) +c;

  for ( i=0;i<PM_L_POINTS;i++ )
  {
    x = lsr->r[i]*pm_co[i];
    y = lsr->r[i]*pm_si[i];
    if ( !lsr->bad[i] && sig* ( si*x-co*y+c ) >0 )
    {
      rx[rn] = x;
      ry[rn++] = y;
    }//if
    if ( !lsa->bad[i] )
    {
      x = lsa->r[i]*pm_co[i];
      y = lsa->r[i]*pm_si[i];
      ax[an] = x*co-y*si+lsa->rx;
      ay[an] = x*si+y*co+lsa->ry;
      if ( ay[an]>0 )
      {
        an++;
      }
    }//if
  }//for i

  dsum = 0;n=0;
  for ( i=0;i<an;i++ )
  {
    dmin = 10000;
    for ( j=0;j<rn;j++ )
    {
      dx = rx[j]-ax[i];
      dy = ry[j]-ay[i];
      d = sqrt ( dx*dx+dy*dy );
      if ( d<dmin )
        dmin = d;
    }//for j
    if ( dmin<10000 )
    {
      n++;
      dsum+=dmin;
    }
  }//for i

  if ( n>0 )
  {
    dsum1 = dsum/ ( PM_TYPE ) n;
    n1    = n;
  }
  else
    return     HUGE_ERROR;

  //now checking the reference scan agains the actual
  dsum = 0;n=0;
  for ( i=0;i<rn;i++ )
  {
    dmin = 10000;
    for ( j=0;j<an;j++ )
    {
      dx = rx[i]-ax[j];
      dy = ry[i]-ay[j];
      d = sqrt ( dx*dx+dy*dy );
      if ( d<dmin )
        dmin = d;
    }//for j
    if ( dmin<10000 )
    {
      n++;
      dsum+=dmin;
    }
  }//for i

  if ( n>0 )
  {
    dsum = dsum/ ( PM_TYPE ) n;
  }
  else
    return     HUGE_ERROR;

  cout<<"pm_error_index: "<<n1<<" "<<dsum1<<" "<<n<<" "<<dsum<<endl;

  if ( n1>MIN_POINTS && n>MIN_POINTS )
  {
    if ( dsum1>dsum )
      return dsum1; //return the larger one
    else
      return dsum;
  }
  return     HUGE_ERROR;
}

//-------------------------------------------------------------------------
//estimates the covariance matrix(c11,c12,c22,c33) (x,y,th) of
//a scan match based on an error index (err-depends on how good the
//match is), and the angle of the corridor if it is a corridor
// for non corridors cov matrix is diagonal
void PolarMatcher::pm_cov_est ( PM_TYPE err, double *c11,double *c12, double *c22, double *c33,
                  bool corridor, PM_TYPE corr_angle )
{
#define SQ(x) ((x)*(x))
  const double cov_x  = SQ ( 20 ); //10       // cm
  const double cov_y  = SQ ( 20 );//10      // cm
  const double cov_th = SQ ( 4.0*M_PI/180.0 ); // 2 deg
  //for corridors
  const double cov_along   = SQ ( 400 );   // cm
  const double cov_across  = SQ ( 30 ); // cm

  err = err-5;
  if ( err<1 )
    err = 1;
  if ( corridor ) //was the actual scan taken of a corridor?
  {
    double co = cos ( corr_angle );
    double si = sin ( corr_angle );
    *c11 = err* ( SQ ( co ) *cov_along+SQ ( si ) *cov_across );
    *c12 = err* ( -co*si* ( -cov_along+cov_across ) );
    *c22 = err* ( SQ ( si ) *cov_along+SQ ( co ) *cov_across );
    *c33 = err*cov_th;
  }//if
  else
  {
    *c12 = 0;
    *c11 = err*cov_x;
    *c22 = err*cov_y;
    *c33 = err*cov_th;
  }//else
}//pm_cov_est

//-------------------------------------------------------------------------
// does scan matching using the equations for translation and orietation
//estimation as in Lu & milios, however our matching bearing association rule
//is used together with our association filter.
//have to do an angle search othervise doesn't converge to 0!
//weights implemented!
PM_TYPE PolarMatcher::pm_psm_c ( PMScan *lsr,PMScan *lsa )
{
//   #define GR //comment out if no graphics necessary
  PMScan  act(PM_L_POINTS),  ref(PM_L_POINTS);//copies of actual and reference scans
  PM_TYPE rx,ry,rth,ax,ay,ath;//robot pos at ref and actual scans
  PM_TYPE t13,t23;
  PM_TYPE new_r[PM_L_POINTS];//interpolated r at measurement bearings
  int     new_bad[PM_L_POINTS];//bad flags of the interpolated range readings
  PM_TYPE C = 70*70;//weighting factor; see dudek00
  int     n = 0;//number of valid points
  int       iter,i,small_corr_cnt=0;
  PM_TYPE   abs_err=0,dx=0,dy=0,dth=0;//match error, actual scan corrections

  act = *lsa;
  ref = *lsr;

  rx =  ref.rx; ry = ref.ry; rth = ref.th;
  ax =  act.rx; ay = act.ry; ath = act.th;

  //transformation of actual scan laser scanner coordinates into reference
  //laser scanner coordinates
  t13 =  cos(rth) * ax + sin(rth) * ay - sin (rth) * ry - rx * cos(rth);
  t23 = -sin(rth) * ax + cos(rth) * ay - cos (rth) * ry + rx * sin(rth);

  ref.rx = 0;   ref.ry = 0;   ref.th = 0;
  act.rx = t13; act.ry = t23; act.th = ath-rth;

  ax = act.rx; ay = act.ry; ath = act.th;
  //from now on act.rx,.. express the lasers position in the ref frame

  iter = -1;
  while ( ++iter<PM_MAX_ITER && small_corr_cnt<3 ) //has to be 5 small corrections before stop
  {
    if ( ( fabs ( dx ) +fabs ( dy ) +fabs ( dth ) ) <PM_STOP_COND )
      small_corr_cnt++;
    else
      small_corr_cnt=0;

    act.rx = ax;act.ry = ay;act.th = ath;
    // convert range readings into ref frame
    pm_scan_project(&act,  new_r, new_bad);

    //---------------ORIENTATION SEARCH-----------------------------------
    //search for angle correction using crosscorrelation
    if ( iter%4 ==3 ) //(iter%2==1)
    {
       dth = pm_orientation_search(&ref, new_r, new_bad);
       ath += dth;
       continue;
    }

    //------------------------------------------translation-------------
    if ( iter>10 )
      C = 100;

    // do the weighted linear regression on the linearized ...
    // include angle as well
    PM_TYPE dr;

    //computation of the new dx1,dy1,dtheta1
    PM_TYPE X=0,Y=0,Xp=0,Yp=0,W=0,w;
    PM_TYPE sxx=0,sxy=0,syx=0, syy=0;
    PM_TYPE meanpx,meanpy,meanppx,meanppy;
    meanpx = 0;meanpy = 0;
    meanppx= 0;meanppy= 0;

    abs_err=0;
    n=0;
    for ( i=0;i<PM_L_POINTS;i++ )
    {
      dr = ref.r[i]-new_r[i];
      abs_err += fabs ( dr );
      //weight calculation
      if ( ref.bad[i]==0 && new_bad[i]==0 && new_r[i]<PM_MAX_RANGE && new_r[i]>10.0 && fabs ( dr ) <PM_MAX_ERROR )
      {
        // do the cartesian calculations....

        PM_TYPE x,y,xp,yp;
        x = new_r[i]*pm_co[i];//actual
        y = new_r[i]*pm_si[i];
        xp = ref.r[i]*pm_co[i]; //could speed up this..
        yp = ref.r[i]*pm_si[i];//reference

        w = C/ ( dr*dr+C );

        W += w;
        X += w*x;
        Y += w*y;
        Xp+= w*xp;
        Yp+= w*yp;

        sxx += w*xp*x;
        sxy += w*xp*y;
        syx += w*yp*x;
        syy += w*yp*y;

        n++;
      }
    }//for
    if (n < PM_MIN_VALID_POINTS || W < 0.01 ) //are there enough points?
    {
      cerr <<"pm_linearized_match: ERROR not enough points " << n << " " << W << endl;
      throw 1;//not enough points
    }

    dth = atan2 ( - ( X*Yp - Y*Xp + W* ( sxy-syx ) ),- ( X*Xp+Y*Yp-W* ( sxx+syy ) ) );
    dx  = ( Xp-X*cos ( dth ) +Y*sin ( dth ) ) /W;
    dy  = ( Yp-X*sin ( dth )-Y*cos ( dth ) ) /W;

    //correction thanks to Alan Zhang:
    PM_TYPE ax_old,ay_old;
    ax_old = ax;
    ay_old = ay;

    ax = cos ( dth ) *ax_old - sin ( dth ) *ay_old + dx;
    ay = sin ( dth ) *ax_old + cos ( dth ) *ay_old + dy;
    ath+= dth;

    dth *=PM_R2D;
  }//for iter

  lsa->rx =ax;lsa->ry=ay;lsa->th=ath;
  return ( abs_err/n );
}//pm_psm_c

//-------------------------------------------------------------------------
// minimizes least square error through changing lsa->rx, lsa->ry,lsa->th
// this looks for angle too, like pm_linearized_match_proper,execept it
// fits a parabola to the error when searching for the angle and interpolates.
PM_TYPE PolarMatcher::pm_psm ( PMScan *lsr,PMScan *lsa )
{
  PMScan    act(PM_L_POINTS),  ref(PM_L_POINTS);//copies of actual and reference scans
  PM_TYPE   rx,ry,rth,ax,ay,ath;//robot pos at ref and actual scans
  PM_TYPE   t13,t23;
  PM_TYPE   new_r[PM_L_POINTS];//interpolated r at measurement bearings
  int       new_bad[PM_L_POINTS];//bad flags of the interpolated range readings
  PM_TYPE   C = 70*70;//weighting factor; see dudek00
  int       iter,small_corr_cnt=0;
  PM_TYPE   dx=0,dy=0,dth=0;//match error, actual scan corrections
  PM_TYPE   avg_err = 100000000.0;

  act = *lsa;
  ref = *lsr;

  rx =  ref.rx; ry = ref.ry; rth = ref.th;
  ax =  act.rx; ay = act.ry; ath = act.th;

  //transformation of actual scan laser scanner coordinates into reference
  //laser scanner coordinates
  t13 =  cos(rth) * ax + sin(rth) * ay-sin (rth) * ry - rx * cos (rth);
  t23 = -sin(rth) * ax + cos(rth) * ay-cos (rth) * ry + rx * sin (rth);

  ref.rx = 0;   ref.ry = 0;   ref.th = 0;
  act.rx = t13; act.ry = t23; act.th = ath-rth;

  ax = act.rx; ay = act.ry; ath = act.th;
  //from now on act.rx,.. express the lasers position in the ref frame

  iter = -1;
  while ( ++iter<PM_MAX_ITER && small_corr_cnt<3 ) //has to be 5 small corrections before stop
  {
    if ( ( fabs ( dx ) +fabs ( dy ) +fabs ( dth ) ) <PM_STOP_COND )
      small_corr_cnt++;
    else
      small_corr_cnt=0;

    act.rx = ax;act.ry = ay;act.th = ath;
    pm_scan_project(&act,  new_r, new_bad);

    //---------------ORIENTATION SEARCH-----------------------------------
    //search for angle correction using crosscorrelation
    if ( iter%2 ==1 )
    {
       dth = pm_orientation_search(&ref, new_r, new_bad);
       ath += dth;
       continue;
    }

    //------------------------------------------translation-------------
    //reduce C with time to consider only the best matches
    if ( iter>10 )
      C = 100;

    avg_err = pm_translation_estimation(&ref, new_r, new_bad, C, &dx, &dy);

    ax += dx;
    ay += dy;

//    //for SIMULATION iteration results..

  }//while iter

  lsa->rx =ax;lsa->ry=ay;lsa->th=ath;
  return ( avg_err);
}//pm_linearized_match_int_angle

//-------------------------------------------------------------------------
//calculate distance of the point (x3,y3) from a line (defined by (x1,y1)
//and (x2,y2)). Returns the distance to the line or -1 if the
//projection of (x3,y3) falls outside the line segment defined by (x1,y1)
//and (x2,y2).
//The projection of (x3,y3) onto the line is also returned in x,y
PM_TYPE PolarMatcher::point_line_distance ( PM_TYPE x1, PM_TYPE y1, PM_TYPE x2, PM_TYPE y2,
                              PM_TYPE x3, PM_TYPE y3,PM_TYPE *x, PM_TYPE *y )
{
  PM_TYPE ax,ay,t1,D;
  ax = x2-x1;
  ay = y2-y1;
  D =  sqrt ( ax*ax+ay*ay );
  if ( D <0.0001 )
  {
    cerr <<"point_line_distance: unexpected D:" <<D<<endl;
    return -1;
  }
  t1 =  - ( -ax*x3 + ay*y1 + ax*x1 - ay*y3 ) / ( ax*ax+ay*ay );
  if ( t1<0 || t1>1 )   // projection falls outside the line segment
  {
    return -1;
  }
  *x = x1+t1*ax;
  *y = y1+t1*ay;
  return ( sqrt ( ( x3-*x ) * ( x3-*x ) + ( y3-*y ) * ( y3-*y ) ) );//distance of line to p.
}//  point_line_distance

//-------------------------------------------------------------------------
//! projects the current (active) scan act to the base (ref.) (0,0,0) coordinate frame
//! returns in new_r interpolated r at measurement bearings
//! returns in new_bad bad flags of the interpolated range readings
void PolarMatcher::pm_scan_project(const PMScan *act,  PM_TYPE   *new_r,  int *new_bad)
{
    PM_TYPE   r[PM_L_POINTS],fi[PM_L_POINTS];//actual scan in ref. coord. syst.
    PM_TYPE   x,y;
    int       i;
    PM_TYPE   delta;

    // convert range readings into ref frame
    // this can be speeded up, by connecting it with the interpolation
    for ( i=0;i<PM_L_POINTS;i++ )
    {
      delta   = act->th + pm_fi[i];
      x       = act->r[i]*cos ( delta ) + act->rx;
      y       = act->r[i]*sin ( delta ) + act->ry;
      r[i]    = sqrt ( x*x+y*y );
      fi[i]   = atan2 ( y,x );
      //handle discontinuity at pi
      if(x<0 && y<0)
        fi[i] += 2.0*M_PI;

      new_r[i]  = 10000;//initialize big interpolated r;
      new_bad[i]= PM_EMPTY;//for interpolated r;

    }//for i

    //------------------------INTERPOLATION------------------------
    //calculate/interpolate the associations to the ref scan points
    //algorithm ignores crosings at 0 and 180 -> to make it faster
    for ( i=1;i<PM_L_POINTS;i++ )
    {
      //i points to the angles in the actual scan

      // i and i-1 has to be in the same segment, both shouldn't be bad
      // and they should be larger than the minimum angle
      if ( act->seg[i] != 0 && act->seg[i] == act->seg[i-1] && !act->bad[i] &&
              !act->bad[i-1] /* && fi[i]>PM_FI_MIN && fi[i-1]>PM_FI_MIN*/ )
      {
        //calculation of the "whole" parts of the angles
        int j0,j1;
        PM_TYPE r0,r1,a0,a1;
        bool occluded;
        if ( fi[i]>fi[i-1] ) //are the points visible?
        {
          //visible
          occluded = false;
          a0  = fi[i-1];
          a1  = fi[i];

          j0  =  (int) ceil ( ( fi[i-1] - PM_FI_MIN ) /PM_DFI );
          j1  =  (int) floor ( ( fi[i] - PM_FI_MIN ) /PM_DFI );
          r0  = r[i-1];
          r1  = r[i];
        }
        else
        {
          //invisible - still have to calculate to filter out points which
          occluded = true; //are covered up by these!
          //flip the points-> easier to program
          a0  = fi[i];
          a1  = fi[i-1];

          j0  =  (int) ceil ( ( fi[i] - PM_FI_MIN ) /PM_DFI );
          j1  =  (int) floor ( ( fi[i-1] - PM_FI_MIN ) /PM_DFI );
          r0  = r[i];
          r1  = r[i-1];
        }
        //here fi0 is always smaller than fi1!

        //interpolate for all the measurement bearings beween fi0 and fi1
        while ( j0<=j1 ) //if at least one measurement point difference, then ...
        {
         // cout <<( ( PM_TYPE ) j0*PM_DFI )<<" "<<( ( ( PM_TYPE ) j0*PM_DFI +PM_FI_MIN )-a0 )<<endl;
          PM_TYPE ri = ( r1-r0 ) / ( a1-a0 ) * ( ( ( PM_TYPE ) j0*PM_DFI+PM_FI_MIN )-a0 ) +r0;

          //if fi0 -> falls into the measurement range and ri is shorter
          //than the current range then overwrite it
          if ( j0>=0 && j0<PM_L_POINTS && new_r[j0]>ri )
          {
            new_r[j0]    = ri;//overwrite the previous reading
            new_bad[j0] &=~PM_EMPTY;//clear the empty flag
            if ( occluded ) //check if it was occluded
              new_bad[j0] = new_bad[j0]|PM_OCCLUDED;//set the occluded flag
            else
              new_bad[j0] = new_bad[j0]&~PM_OCCLUDED;
            //the new range reading also it has to inherit the other flags
            new_bad[j0] |= act->bad[i];//superfluos - since act.bad[i] was checked for 0
            new_bad[j0] |= act->bad[i-1];//superfluos - since act.bad[i-1] was checked for 0

          }
          j0++;//check the next measurement angle!
        }//while
      }//if act
    }//for i
}//pm_scan_project

//-------------------------------------------------------------------------
//! Function estimating the orientation of the current scan represented with range readings
//! new_r and flags new_bad with respect to the reference scan ref.
//! Assuming that the current and ref scan were taken at the same position, to an orientation
//! change of the current scan a left or right shift of the scan ranges corresponds.
//! The functions estimates the orientation by finding that shift which minimizes the
//! difference between the current and ref. scan. The orientation estimate is then
//! refined using interpolation. 
PM_TYPE PolarMatcher::pm_orientation_search(const PMScan *ref, const PM_TYPE *new_r, const int *new_bad)
{
      int i;
      int       window       = PM_SEARCH_WINDOW;//20;//+- width of search for correct orientation
      PM_TYPE   dth=0;//\actual scan corrections

      //pm_fi,ref.r - reference points
      PM_TYPE e,err[PM_L_POINTS];       // the error rating
      PM_TYPE beta[PM_L_POINTS];// angle for the corresponding error
      PM_TYPE n;
      int k=0;

      for ( int di=-window;di<=window;di++ )
      {
        n=0;e=0;

        int min_i,max_i;
        if ( di<=0 )
          {min_i = -di;max_i=PM_L_POINTS;}
        else
          {min_i = 0;max_i=PM_L_POINTS-di;}

        for ( i=min_i;i<max_i;i++ ) //searching through the actual points
        {
          //if fi[i],r[i] isn't viewed from the back, isn't moving
          // and isn't a solitary point, then try to associate it ..
          //also fi[i] is within the angle range ...

          if ( !new_bad[i] && !ref->bad[i+di] )
          {
            e += fabs ( new_r[i]-ref->r[i+di] );
            n++;
          }

        }//for i

        if ( n>0 )
          err[k]  = e/n;//don't forget to correct with n!
        else
          err[k]  = 10000;//don't forget to correct with n!
        beta[k] = di;
        k++;
      }//for dfi

      //now search for the global minimum
      //later I can make it more robust
      //assumption: monomodal error function!
      PM_TYPE emin = 1000000;
      int   imin;
      for ( i=0;i<k;i++ )
        if ( err[i]<emin )
        {
          emin = err[i];
          imin = i;
        }
      if ( err[imin]>=10000 )
      {
        cerr <<"Polar Match: orientation search failed" <<err[imin]<<endl;
        throw 1;
      }
      dth = beta[imin]*PM_DFI;

      //interpolation
      if ( imin>=1 && imin< ( k-1 ) ) //is it not on the extreme?
      {
        //lets try interpolation
        PM_TYPE D = err[imin-1]+err[imin+1]-2.0*err[imin];
        PM_TYPE d=1000;
        if ( fabs ( D ) >0.01 && err[imin-1]>err[imin] && err[imin+1]>err[imin] )
          d= ( err[imin-1]-err[imin+1] ) /D/2.0;

        if ( fabs ( d ) <1 )
          dth+=d*PM_DFI;
      }//if

     return(dth);
}//pm_orientation_search

PM_TYPE PolarMatcher::pm_translation_estimation(const PMScan *ref, const PM_TYPE *new_r, const int *new_bad, PM_TYPE C, PM_TYPE *dx, PM_TYPE *dy)
{
  // do the weighted linear regression on the linearized ...
  // include angle as well
  int i;
  PM_TYPE hi1, hi2,hwi1,hwi2, hw1=0,hw2=0,hwh11=0;
  PM_TYPE hwh12=0,hwh21=0,hwh22=0,w;
  PM_TYPE dr;
  PM_TYPE abs_err = 0;
  int     n = 0;
  for ( i=0;i<PM_L_POINTS;i++ )
  {
    dr = ref->r[i]-new_r[i];
    abs_err += fabs ( dr );

    //weight calculation
    if (ref->bad[i] == 0            && 
        new_bad[i]  == 0            && 
        new_r[i]    <  PM_MAX_RANGE && 
        new_r[i]    > 10.0          && 
        fabs(dr)    < PM_MAX_ERROR  )
    {
      //weighting according to DUDEK00

      w = C / (dr*dr + C);
      n++;

      //proper calculations of the jacobian
      hi1 = pm_co[i];//xx/new_r[i];//this the correct
      hi2 = pm_si[i];//yy/new_r[i];

      hwi1 = hi1*w;
      hwi2 = hi2*w;

      //par = (H^t*W*H)^-1*H^t*W*dr
      hw1 += hwi1*dr;//H^t*W*dr
      hw2 += hwi2*dr;

      //H^t*W*H
      hwh11 += hwi1*hi1;
      hwh12 += hwi1*hi2;
      hwh21 += hwi2*hi1; //should take adv. of symmetricity!!
      hwh22 += hwi2*hi2;
    }
    else
    {
/*
      if (ref->bad[i] != 0) printf("rb ");
      if (new_bad[i]  != 0) printf("nb ");
      if (new_r[i]    <  PM_MAX_RANGE) printf("MR ");
      if (fabs(dr)    < PM_MAX_ERROR) printf("ME ");
      printf(".\n");
  */
    }
  }//for i
  if ( n<PM_MIN_VALID_POINTS ) //are there enough points?
  {
    cerr <<"(i) pm_translation_estimation: ERROR not enough points" << n <<endl;
    throw 1;//not enough points
  }

  //calculation of inverse
  PM_TYPE D;//determinant
  PM_TYPE inv11,inv21,inv12,inv22;//inverse matrix

  D = hwh11*hwh22-hwh12*hwh21;
  if ( D<0.001 )
  {
    cerr <<"pm_linearized_match: ERROR determinant to small! "<<D<<endl;
    throw 1;
  }
  inv11 =  hwh22/D;
  inv12 = -hwh12/D;
  inv21 = -hwh12/D;
  inv22 =  hwh11/D;

  *dx = inv11*hw1+inv12*hw2;
  *dy = inv21*hw1+inv22*hw2;
  return(abs_err/n);
}//pm_translation_estimation

