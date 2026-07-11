#include "youbot_kin.h"

#include <math.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <vector>
#include <float.h>

#include <iostream>
#include <sstream>

#include "EulerAngle.h"
#include "Orientation.h"

#include "TMPLibrary/TMPLibrary.h"

namespace youbot_kinematics {
  
  // namespace {
    // const double ZERO_THRESH = 0.00000001;
    // int SIGN(double x) {
    //   return (x > 0) - (x < 0);
    // }
    // const double PI = M_PI;

  //   //#define UR10_PARAMS
  //   #ifdef UR10_PARAMS
  //   const double d1 =  0.1273;
  //   const double a2 = -0.612;
  //   const double a3 = -0.5723;
  //   const double d4 =  0.163941;
  //   const double d5 =  0.1157;
  //   const double d6 =  0.0922;
  //   #endif

  //   //#define UR5_PARAMS
  //   #ifdef UR5_PARAMS
  //   const double d1 =  0.089159;
  //   const double a2 = -0.42500;
  //   const double a3 = -0.39225;
  //   const double d4 =  0.10915;
  //   const double d5 =  0.09465;
  //   const double d6 =  0.0823;
  //   #endif

  //   #define UR5E_PARAMS
  //   #ifdef UR5E_PARAMS
  //   const double d1 =  0.1625;
  //   const double a2 = -0.42500;
  //   const double a3 = -0.39225;
  //   const double d4 =  0.1333;
  //   const double d5 =  0.0997;
  //   const double d6 =  0.0996;
  //   #endif
    
  //   //#define UR3_PARAMS
  //   #ifdef UR3_PARAMS
  //   const double d1 =  0.1519;
  //   const double a2 = -0.24365;
  //   const double a3 = -0.21325;
  //   const double d4 =  0.11235;
  //   const double d5 =  0.08535;
  //   const double d6 =  0.0819;
  //   #endif
  // }

  // void forward(const double* q, double* T) {
  //   double s1 = sin(*q), c1 = cos(*q); q++;
  //   double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
  //   //double s3 = sin(*q), c3 = cos(*q); q23 += *q; q234 += *q; q++;
  //   double s4 = sin(*q), c4 = cos(*q); q234 += *q; q++;
  //   double s5 = sin(*q), c5 = cos(*q); q++;
  //   double s6 = sin(*q), c6 = cos(*q); 
  //   double s23 = sin(q23), c23 = cos(q23);
  //   double s234 = sin(q234), c234 = cos(q234);
  //   *T = c234*c1*s5 - c5*s1; T++;
  //   *T = c6*(s1*s5 + c234*c1*c5) - s234*c1*s6; T++;
  //   *T = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6; T++;
  //   *T = d6*c234*c1*s5 - a3*c23*c1 - a2*c1*c2 - d6*c5*s1 - d5*s234*c1 - d4*s1; T++;
  //   *T = c1*c5 + c234*s1*s5; T++;
  //   *T = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6; T++;
  //   *T = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1; T++;
  //   *T = d6*(c1*c5 + c234*s1*s5) + d4*c1 - a3*c23*s1 - a2*c2*s1 - d5*s234*s1; T++;
  //   *T = -s234*s5; T++;
  //   *T = -c234*s6 - s234*c5*c6; T++;
  //   *T = s234*c5*s6 - c234*c6; T++;
  //   *T = d1 + a3*s23 + a2*s2 - d5*(c23*c4 - s23*s4) - d6*s5*(c23*s4 + s23*c4); T++;
  //   *T = 0.0; T++; *T = 0.0; T++; *T = 0.0; T++; *T = 1.0;
  // }

  // void forward_all(const double* q, double* T1, double* T2, double* T3, 
  //                                   double* T4, double* T5, double* T6) {
  //   double s1 = sin(*q), c1 = cos(*q); q++; // q1
  //   double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++; // q2
  //   //double s3 = sin(*q), c3 = cos(*q); 
  //   q23 += *q; q234 += *q; q++; // q3
  //   q234 += *q; q++; // q4
  //   double s5 = sin(*q), c5 = cos(*q); q++; // q5
  //   double s6 = sin(*q), c6 = cos(*q); // q6
  //   double s23 = sin(q23), c23 = cos(q23);
  //   double s234 = sin(q234), c234 = cos(q234);

  //   if(T1 != NULL) {
  //     *T1 = c1; T1++;
  //     *T1 = 0; T1++;
  //     *T1 = s1; T1++;
  //     *T1 = 0; T1++;
  //     *T1 = s1; T1++;
  //     *T1 = 0; T1++;
  //     *T1 = -c1; T1++;
  //     *T1 = 0; T1++;
  //     *T1 =       0; T1++;
  //     *T1 = 1; T1++;
  //     *T1 = 0; T1++;
  //     *T1 =d1; T1++;
  //     *T1 =       0; T1++;
  //     *T1 = 0; T1++;
  //     *T1 = 0; T1++;
  //     *T1 = 1; T1++;
  //   }

  //   if(T2 != NULL) {
  //     *T2 = c1*c2; T2++;
  //     *T2 = -c1*s2; T2++;
  //     *T2 = s1; T2++;
  //     *T2 =a2*c1*c2; T2++;
  //     *T2 = c2*s1; T2++;
  //     *T2 = -s1*s2; T2++;
  //     *T2 = -c1; T2++;
  //     *T2 =a2*c2*s1; T2++;
  //     *T2 =         s2; T2++;
  //     *T2 = c2; T2++;
  //     *T2 = 0; T2++;
  //     *T2 =   d1 + a2*s2; T2++;
  //     *T2 =               0; T2++;
  //     *T2 = 0; T2++;
  //     *T2 = 0; T2++;
  //     *T2 =                 1; T2++;
  //   }

  //   if(T3 != NULL) {
  //     *T3 = c23*c1; T3++;
  //     *T3 = -s23*c1; T3++;
  //     *T3 = s1; T3++;
  //     *T3 =c1*(a3*c23 + a2*c2); T3++;
  //     *T3 = c23*s1; T3++;
  //     *T3 = -s23*s1; T3++;
  //     *T3 = -c1; T3++;
  //     *T3 =s1*(a3*c23 + a2*c2); T3++;
  //     *T3 =         s23; T3++;
  //     *T3 = c23; T3++;
  //     *T3 = 0; T3++;
  //     *T3 =     d1 + a3*s23 + a2*s2; T3++;
  //     *T3 =                    0; T3++;
  //     *T3 = 0; T3++;
  //     *T3 = 0; T3++;
  //     *T3 =                                     1; T3++;
  //   }

  //   if(T4 != NULL) {
  //     *T4 = c234*c1; T4++;
  //     *T4 = s1; T4++;
  //     *T4 = s234*c1; T4++;
  //     *T4 =c1*(a3*c23 + a2*c2) + d4*s1; T4++;
  //     *T4 = c234*s1; T4++;
  //     *T4 = -c1; T4++;
  //     *T4 = s234*s1; T4++;
  //     *T4 =s1*(a3*c23 + a2*c2) - d4*c1; T4++;
  //     *T4 =         s234; T4++;
  //     *T4 = 0; T4++;
  //     *T4 = -c234; T4++;
  //     *T4 =                  d1 + a3*s23 + a2*s2; T4++;
  //     *T4 =                         0; T4++;
  //     *T4 = 0; T4++;
  //     *T4 = 0; T4++;
  //     *T4 =                                                  1; T4++;
  //   }

  //   if(T5 != NULL) {
  //     *T5 = s1*s5 + c234*c1*c5; T5++;
  //     *T5 = -s234*c1; T5++;
  //     *T5 = c5*s1 - c234*c1*s5; T5++;
  //     *T5 =c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1; T5++;
  //     *T5 = c234*c5*s1 - c1*s5; T5++;
  //     *T5 = -s234*s1; T5++;
  //     *T5 = - c1*c5 - c234*s1*s5; T5++;
  //     *T5 =s1*(a3*c23 + a2*c2) - d4*c1 + d5*s234*s1; T5++;
  //     *T5 =                           s234*c5; T5++;
  //     *T5 = c234; T5++;
  //     *T5 = -s234*s5; T5++;
  //     *T5 =                          d1 + a3*s23 + a2*s2 - d5*c234; T5++;
  //     *T5 =                                                   0; T5++;
  //     *T5 = 0; T5++;
  //     *T5 = 0; T5++;
  //     *T5 =                                                                                 1; T5++;
  //   }

  //   if(T6 != NULL) {
  //     *T6 =   c6*(s1*s5 + c234*c1*c5) - s234*c1*s6; T6++;
  //     *T6 = - s6*(s1*s5 + c234*c1*c5) - s234*c1*c6; T6++;
  //     *T6 = c5*s1 - c234*c1*s5; T6++;
  //     *T6 =d6*(c5*s1 - c234*c1*s5) + c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1; T6++;
  //     *T6 = - c6*(c1*s5 - c234*c5*s1) - s234*s1*s6; T6++;
  //     *T6 = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1; T6++;
  //     *T6 = - c1*c5 - c234*s1*s5; T6++;
  //     *T6 =s1*(a3*c23 + a2*c2) - d4*c1 - d6*(c1*c5 + c234*s1*s5) + d5*s234*s1; T6++;
  //     *T6 =                                       c234*s6 + s234*c5*c6; T6++;
  //     *T6 = c234*c6 - s234*c5*s6; T6++;
  //     *T6 = -s234*s5; T6++;
  //     *T6 =                                                      d1 + a3*s23 + a2*s2 - d5*c234 - d6*s234*s5; T6++;
  //     *T6 =                                                                                                   0; T6++;
  //     *T6 = 0; T6++;
  //     *T6 = 0; T6++;
  //     *T6 =                                                                                                                                            1; T6++;
  //   }
  // }

  // int inverse(const double* T, double* q_sols, double q6_des) {

  // int inverse(Vector6d s) {
                //   int inverse(const double* T, double* q_sols, double q6_des) {
                //     using namespace std;
                //     Eigen::VectorXd a(5);
                //     a << 0.033, 0.155, 0.135, 0.0, 0.0;

                //     Eigen::VectorXd d(5);
                //     d << 0.147, 0., 0., 0.0, 0.2175;

                //     double T00 = *T; 
                //     T++; 
                //     // double T01 = *T; 
                //     T++; 
                //     // double T02 = *T; 
                //     T++; 
                //     double T03 = *T; 
                //     T++; 
                //     double T10 = *T; 
                //     T++; 
                //     // double T11 = *T; 
                //     T++; 
                //     // double T12 = *T; 
                //     T++; 
                //     double T13 = *T; 
                //     T++; 
                //     double T20 = *T; 
                //     T++; 
                //     double T21 = *T; 
                //     T++; 
                //     double T22 = *T; 
                //     T++; 
                //     double T23 = *T;

                //   // ConfigurationsOfManipulator conf_manip;

                //   double px = T03, py = T13, pz = T23;

                //   double yaw=atan2(T10,T00);
                //   double pitch=atan2(-T20,sqrt(T21*T21+T22*T22));
                //   double roll=atan2(T21,T22);

                //   Eigen::Matrix3d pr;
                //   pr = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                //       * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitY())
                //       * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX());
                //   // theta_1
                //   double theta_1_I = atan2(py, px);
                //   double theta_1_II = atan2(-py, -px);

                //   //theta_2
                //   double s5 = pr(0, 0) * sin(theta_1_I) - pr(1, 0) * cos(theta_1_I);
                //   double c5 = pr(0, 1) * sin(theta_1_I) - pr(1, 1) * cos(theta_1_I);
                //   double theta_5_I = atan2(s5, c5);
                //   s5 = pr(0, 0) * sin(theta_1_II) - pr(1, 0) * cos(theta_1_II);
                //   c5 = pr(0, 1) * sin(theta_1_II) - pr(1, 1) * cos(theta_1_II);
                //   double theta_5_II = atan2(s5, c5);

                //   // theta_234
                //   double c234 = -pr(2, 2);
                //   double s234 = pr(0, 2) * cos(theta_1_I) + pr(1, 2) * sin(theta_1_I);
                //   double theta_234_I = atan2(s234, c234);
                //   s234 = pr(0, 2) * cos(theta_1_II) + pr(1, 2) * sin(theta_1_II);
                //   double theta_234_II = atan2(s234, c234);

                //   // theta_3
                //   Eigen::Vector3d v = get_vector_1to4_frame(theta_1_I, pr, Eigen::Vector3d(T03, T13, T23));
                //   double xr = v(0), yr = v(1), zr = v(2);
                //   double numerator_0 = xr * xr + yr * yr + zr * zr - a(1) * a(1) - a(2) * a(2);
                //   double denominator_0 = 2 * a(1) * a(2);
                //   double cosq3_I = numerator_0 / denominator_0;

                //   if (cosq3_I <= 1) {
                //     double theta_3_I = -atan2(sqrt(1.0 - cosq3_I * cosq3_I), cosq3_I);
                //     double theta_3_II = atan2(sqrt(1.0 - cosq3_I * cosq3_I), cosq3_I);

                //     double phi_I = atan2(yr, xr);
                //     double beta_I = atan2(a(2) * sin(abs(theta_3_I)), a(1) + a(2) * cos(abs(theta_3_I)));
                //     double beta_II = atan2(a(2) * sin(abs(theta_3_II)), a(1) + a(2) * cos(abs(theta_3_II)));

                //     double theta_2_I = phi_I + beta_I;
                //     double theta_2_II = phi_I - beta_II;

                //     double theta_4_I = theta_234_I - theta_2_I - theta_3_I;
                //     double theta_4_II = theta_234_I - theta_2_II - theta_3_II;

                //     int hop = 0;
                //     q_sols[hop*5+0] = theta_1_I;
                //     q_sols[hop*5+1] = theta_2_I;
                //     q_sols[hop*5+2] = theta_3_I;
                //     q_sols[hop*5+3] = theta_4_I;
                //     q_sols[hop*5+4] = theta_5_I;

                //     hop = 1;
                //     q_sols[hop*5+0] = theta_1_I;
                //     q_sols[hop*5+1] = theta_2_II;
                //     q_sols[hop*5+2] = theta_3_II;
                //     q_sols[hop*5+3] = theta_4_II;
                //     q_sols[hop*5+4] = theta_5_I;
                    
                //     // conf_manip.qs.row(0) << theta_1_I, theta_2_I, theta_3_I, theta_4_I, theta_5_I,
                //     // conf_manip.qs.row(1) << theta_1_I, theta_2_II, theta_3_II, theta_4_II, theta_5_I;
                //   } else {
                //     std::cout << "IK failed" << std::endl;
                //     return 0;
                //   }

                //   v = get_vector_1to4_frame(theta_1_II, pr, Eigen::Vector3d(px, py, pz));
                //   xr = v(0); yr = v(1); zr = v(2);
                //   xr -= FLT_EPSILON;
                //   numerator_0 = xr * xr + yr * yr + zr * zr - a(1)*a(1) - a(2)*a(2);
                //   denominator_0 = 2 * a(1) * a(2);
                //   double cosq3_II = numerator_0 / denominator_0  - copysign(1.0, numerator_0) * FLT_EPSILON;

                //   if (cosq3_II <= 1) {
                //     double theta_3_III = -atan2(sqrt(1.0 - cosq3_II * cosq3_II), cosq3_II);
                //     double theta_3_IV = atan2(sqrt(1.0 - cosq3_II * cosq3_II), cosq3_II);

                //     double phi_II = atan2(yr, xr);
                //     double beta_III = atan2(a(2) * sin(abs(theta_3_III)), a(1) + a(2) * cos(abs(theta_3_III)));
                //     double beta_IV = atan2(a(2) * sin(abs(theta_3_IV)), a(1) + a(2) * cos(abs(theta_3_IV)));

                //     double theta_2_III = phi_II + beta_III;
                //     double theta_2_IV = phi_II - beta_IV;

                //     double theta_4_III = theta_234_II - theta_2_III - theta_3_III;
                //     double theta_4_IV = theta_234_II - theta_2_IV - theta_3_IV;


                //     int hop = 0;
                //     q_sols[hop*5+0] = theta_1_II;
                //     q_sols[hop*5+1] = theta_2_III;
                //     q_sols[hop*5+2] = theta_3_III;
                //     q_sols[hop*5+3] = theta_4_III;
                //     q_sols[hop*5+4] = theta_5_II;

                //     hop = 1;
                //     q_sols[hop*5+0] = theta_1_II;
                //     q_sols[hop*5+1] = theta_2_IV;
                //     q_sols[hop*5+2] = theta_3_IV;
                //     q_sols[hop*5+3] = theta_4_IV;
                //     q_sols[hop*5+4] = theta_5_II;

                //     // conf_manip.qs.row(2) << theta_1_II, theta_2_III, theta_3_III, theta_4_III, theta_5_II;
                //     // conf_manip.qs.row(3) << theta_1_II, theta_2_IV, theta_3_IV, theta_4_IV, theta_5_II;
                //   } else {
                //     std::cout << "IK failed" << std::endl;
                //     return 0;
                //   }

                //   // Eigen::Matrix<double, 4, 5> qs;    /*!< Matrix for four configurations of manipulator. */
                //   // Eigen::Matrix<double, 4, 1> solves;            /*!< achievable or not */
                //   // qs = Eigen::Matrix<double, 4, 5>::Zero();
                //   // solves = Eigen::Matrix<double, 4, 1>::Ones();

                //   // for(int j = 0; j < 5; j++) {
                //   //   for (int i = 0; i < 4; i++) {
                //   //     qs(i, j) = theta(j) - qs(i, j);
                //   //     qs(i, j) = normalized_angle(qs(i, j), j);
                //   //   }
                //   // }
                //   return 2;
                // }

                // Eigen::Vector3d get_vector_1to4_frame(double theta, Eigen::Matrix<double, 3, 3> pr, Eigen::Vector3d p) {
                //   Eigen::Vector3d v;
                //   Eigen::VectorXd a(5);
                //   a << 0.033, 0.155, 0.135, 0.0, 0.0;

                //   Eigen::VectorXd d(5);
                //   d << 0.147, 0., 0., 0.0, 0.2175;

                //   double px = p(0), py = p(1), pz = p(2);
                //   double c1 = cos(theta), s1 = sin(theta);
                //   double xr = (py - d(4) * pr(1,2)) * s1 + (px - d(4) * pr(0,2)) * c1 - a(0);
                //   double yr = pz - d(0) - d(4) * pr(2, 2);
                //   double zr = 0.0;
                //   v << xr, yr, zr;
                //   return v;
                // }

                // double normalized_angle(double q, int j) {
                //   if (j != 2) {
                //     while (q < 0.0) {
                //       q += 2. * M_PI;
                //     }
                //     while (q >= 2 * M_PI) {
                //       q -= 2. * M_PI;
                //     }
                //   } else {
                //     while (q > 0.0) {
                //       q -= 2. * M_PI;
                //     }
                //     while (q <= -2 * M_PI) {
                //       q += 2. * M_PI;
                //     }
                //    }
                //   return q;
                // }

                                                  // int inverse(const double* T, double* q_sols, double q6_des) {
                                                  // // int num_sols = 0;
                                                  // // double T02 = -*T; T++; double T00 =  *T; T++; double T01 =  *T; T++; double T03 = -*T; T++; 
                                                  // // double T12 = -*T; T++; double T10 =  *T; T++; double T11 =  *T; T++; double T13 = -*T; T++; 
                                                  // // double T22 =  *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 =  *T;

                                                  // // ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
                                                  // // double q1[2];


                                                  // // double px=0,pz=0,py=0;
                                                  // // double theta_base=0, theta_2=0, theta_3=0, theta_4=0;
                                                  // // double theta_link_1=0 , theta_link_2=0;
                                                  // // double theta_link_1p=0, theta_link_2p=0;


                                                  // /* configuration flags for different system configuration (e.g. base without arm)*/
                                                  // /*
                                                  // * Variable for the base.
                                                  // * Here "boost units" is used to set values in OODL, that means you have to set a value and a unit.
                                                  // */

                                                  // /* Variable for the arm. */
                                                  // //GripperBarSpacingSetPoint gripperSetPoint;

                                                  // /* unfold arm 
                                                  // * all of the following constants are empirically determined to move the arm into the desired position 
                                                  // */
                                                  // double radian = 1.;
                                                  // // desiredJointAngle.angle = 2.9624 * radian;

                                                  // // int n =0, m=0;
                                                  // // double grid_spacing_n =0, grid_spacing_m =0, x_cordinate=0, y_cordinate=0, z_cordinate =0;

                                                  // // T[3] = translation[0];
                                                  // // T[7] = translation[1];
                                                  // // T[11] = translation[2];

                                                  // double px = T[3];
                                                  // double py = -T[7];
                                                  // double pz = T[11];   //0.0432;


                                                  // ///////////////////   BASE THETA    ///////////////////////////


                                                  // py = py + 0.03;

                                                  // double theta_base = atan2 (py,px);

                                                  // {

                                                  //   if (py >=0){
                                                  //     theta_base = theta_base + 2.9624;
                                                  //   }
                                                  //     else { 
                                                  //       theta_base = theta_base - 2.9624 + 5.8201;
                                                  //     }
                                                    
                                                    
                                                  //   if (theta_base > 5.8201){
                                                  //   }
                                                    
                                                  //   if (theta_base < 0){
                                                  //   }
                                                    
                                                  // }


                                                  // //////////////////    THETAS FOR ARM-LINK 1 2 3   ///////////

                                                  // double l1 = 0.302 - 0.147;
                                                  // double l2 = 0.437 - 0.302;
                                                  // double l3 = 0.655 - 0.437;


                                                  // double xc = sqrt(px*px +py*py);
                                                  // double zc = pz;
                                                  // double phi_c = 0;



                                                  // // double d = sqrt (xc*xc + zc*zc);

                                                  // // if (d >= 0.500){
                                                  // //   double theta_link_1 = atan2(zc,xc) ;
                                                  // // //break;
                                                  // // }

                                                  // // else if (d == 0.508) {
                                                  // //   double theta_link_1 = atan2(zc,xc) ;
                                                  // // //break;
                                                  // // }

                                                  // // // else if (d > 0.508) {
                                                  // // // }		


                                                  // // else {
                                                        
                                                  //   double xw = xc - l3*cos(phi_c);
                                                  //   double zw = zc - l3*sin(phi_c);
                                                    
                                                  //   double alpha = atan2 (zw,xw);
                                                    
                                                  //   double cos_beta =  (l1*l1 + l2*l2 -xw*xw -zw*zw)/(2*l1*l2);
                                                  //   double sin_beta = sqrt (abs(1 - (cos_beta*cos_beta)));
                                                  //   double theta_link_2 = 3.1416 - atan2 (sin_beta , cos_beta) ;
                                                    
                                                  //   double cos_gama = (xw*xw + zw*zw + l1*l1 - l2*l2)/(2*l1*sqrt(xw*xw + zw*zw));
                                                  //   double sin_gama = sqrt (abs (1 - (cos_gama * cos_gama)));
                                                    
                                                          
                                                  //   double theta_link_1 = alpha - atan2(sin_gama, cos_gama);
                                                    
                                                    
                                                  //   double theta_link_1p = theta_link_1 + 2*atan2 (sin_gama , cos_gama);
                                                  //   double theta_link_2p = - theta_link_2;
                                                    
                                                    
                                                    
                                                    
                                                    
                                                    
                                                  //   double theta_2 = theta_link_1p;
                                                  //   double theta_3 = theta_link_2p;
                                                  //   double theta_4 = (theta_2 + theta_3);
                                                    
                                                    
                                                    
                                                  //   q_sols[0] = theta_base;
                                                  //   q_sols[1] = (2.5988 - theta_2) * radian;
                                                  //   q_sols[2] = (-2.4352 - theta_3) * radian;
                                                  //   q_sols[3] = (1.7318 + theta_4) * radian; 
                                                  //   // (1.7318-0.4 + theta_4) * radian;
                                                  // // }

                                                  // return 1;
                                                  //   //desiredJointAngle.angle = 0.0 * radian;
                                                  //   //myYouBotManipulator->getArmJoint(1).setData(desiredJointAngle);

                                                  // /*	/* fold arm (approx. home position) using empirically determined values for the positions */
                                                  // }
                                                  // }



// using namespace youbot_arm_kinematics;


#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

const double ALMOST_PLUS_ONE = 0.9999999;
const double ALMOST_MINUS_ONE = -0.9999999;


// InverseKinematics::InverseKinematics(
        // const std::vector<double> &min_angles,
        // const std::vector<double> &max_angles,
//         Logger &logger) : logger_(logger)
// {
//     min_angles_ = min_angles;
//     max_angles_ = max_angles;
}


// int CartToJnt(const KDL::JntArray &q_init,
//         const KDL::Frame &p_in,
//         std::vector<KDL::JntArray> &q_out)
// {
//     KDL::JntArray solution;
//     bool bools[] = { true, false };

//     // there are no solutions available yet
//     q_out.clear();


//     // iterate over all redundant solutions
//     for (int i = 0; i < 2; i++) {
//         for (int j = 0; j < 2; j++) {
//             solution = ik(p_in, bools[i], bools[j]);
//             if (isSolutionValid(solution)) q_out.push_back(solution);
//         }
//     }

//     if (q_out.size() > 0) {
//         logger_.write("Inverse kinematics found a solution",
//                 __FILE__, __LINE__);

//         return 1;
//     } else {
//         logger_.write("Inverse kinematics found no solution",
//                 __FILE__, __LINE__);

//         return -1;
//     }
// }


// int inverse(const KDL::Frame& g0,
//         bool offset_joint_1, bool offset_joint_3)
// {
  int inverse(const double* T, double* q_sols, double q6_des) {
    bool offset_joint_1 = true;
    bool offset_joint_3 = true;

    int num_sols = 1;
    double T02 = -*T; T++; double T00 =  *T; T++; double T01 =  *T; T++; double T03 = -*T; T++; 
    double T12 = -*T; T++; double T10 =  *T; T++; double T11 =  *T; T++; double T13 = -*T; T++; 
    double T22 =  *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 =  *T;

    // Parameters from youBot URDF file
    double l0x = 0.024;
    double l0z = 0.096;
    double l1x = 0.033;
    double l1z = 0.019;
    double l2 = 0.155;
    double l3 = 0.135;

    // Distance from arm_link_4 to arm_link_5
    double d = 0.13;

    double j1 = 0.0;
    double j2 = 0.0;
    double j3 = 0.0;
    double j4 = 0.0;
    double j5 = 0.0;

    double alpha = atan2(T10, T00);
    double beta = atan2(-T20, sqrt(T21*T21 + T22*T22));
    double gamma = atan2(T21, T22);


    // Transform from frame 0 to frame 1
    auto g0 = Transformation(Vector3d(T03,
                            T13,
                            T23),
                    EulerAngle(gamma, // x is gamma
                              beta, // y is beta
                              alpha));// z is alpha
    
    auto frame0_to_frame1 = Transformation(Vector3d(-l0x,
                                            0,
                                            -l0z),
                                            EulerAngle(0, 0, 0));
    
    auto g1 = frame0_to_frame1 * g0;
    // KDL::Frame frame0_to_frame1(
    //         KDL::Rotation::Identity(),
    //         KDL::Vector(-l0x, 0, -l0z));
    // KDL::Frame g1 = frame0_to_frame1 * g0;

    // First joint
    j1 = atan2(T13, T03);
    if (offset_joint_1) {
        if (j1 < 0.0) j1 += M_PI;
        else j1 -= M_PI;
    }


    // Transform from frame 1 to frame 2
    auto frame1_to_frame2 = Transformation(Vector3d(-l1x,
                                            0,
                                            -l1z),
                                            EulerAngle(0, 0, -j1));
    
    // // KDL::Frame frame1_to_frame2(
    // //         KDL::Rotation::RPY(0, 0, -j1),
    // //         KDL::Vector(-l1x, 0, -l1z));
    // KDL::Frame g2 = frame1_to_frame2 * g1;

    auto g2 = frame1_to_frame2 * g1;

    // Project the frame into the plane of the arm
    auto g2_proj = projectGoalOrientationIntoArmSubspace(g2);

    // Set all values in the frame that are close to zero to exactly zero
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (fabs(g2_proj.M(i, j)) < 0.000001) g2_proj.M(i, j) = 0;
        }
    }

    // Fifth joint, determines the roll of the gripper (= wrist angle)
    double s1 = sin(j1);
    double c1 = cos(j1);
    double r11 = g1.rotation().matrix()(0, 0);
    double r12 = g1.rotation().matrix()(0, 1);
    double r21 = g1.rotation().matrix()(1, 0);
    double r22 = g1.rotation().matrix()(1, 1);
    j5 = atan2(r21 * c1 - r11 * s1, r22 * c1 - r12 * s1);

    // The sum of joint angles two to four determines the overall "pitch" of the
    // end effector
    double r13 = g2_proj.M(0, 2);
    double r33 = g2_proj.M(2, 2);
    double j234 = atan2(r13, r33);

    auto p2 = g2_proj.translation();

    // In the arm's subplane, offset from the end-effector to the fourth joint
    // p2.x(p2[0] - d * sin(j234));
    // p2.z(p2[2] - d * cos(j234));

    p2[0] = p2[0] - d * sin(j234);
    p2[2] = p2[2] - d * cos(j234);


    // Check if the goal position can be reached at all
    if ((l2 + l3) < sqrt((p2.x() * p2.x()) + (p2.z() * p2.z()))) {
        // return KDL::JntArray();
        return 0;
    }

    // Third joint
    double l_sqr = (p2.x() * p2.x()) + (p2.z() * p2.z());
    double l2_sqr = l2 * l2;
    double l3_sqr = l3 * l3;
    double j3_cos = (l_sqr - l2_sqr - l3_sqr) / (2.0 * l2 * l3);

    if (j3_cos > ) j3 = 0.0;
    else if (j3_cos < ALMOST_MINUS_ONE) j3 = M_PI;
    else j3 = atan2(sqrt(1.0 - (j3_cos * j3_cos)), j3_cos);

    if (offset_joint_3) j3 = -j3;


    // Second joint
    double t1 = atan2(p2.z(), p2.x());
    double t2 = atan2(l3 * sin(j3), l2 + l3 * cos(j3));
    j2 = M_PI_2 - t1 - t2;


    // Fourth joint, determines the pitch of the gripper
    j4 = j234 - j2 - j3;


    // This IK assumes that the arm points upwards, so we need to consider
    // the offsets to the real home position
    double offset1 = DEG_TO_RAD( 169.0);
    double offset2 = DEG_TO_RAD(  65.0);
    double offset3 = DEG_TO_RAD(-146.0);
    double offset4 = DEG_TO_RAD( 102.5);
    double offset5 = DEG_TO_RAD( 167.5);

    // KDL::JntArray solution(5);
    // solution(0) = offset1 - j1;
    // solution(1) = j2 + offset2;
    // solution(2) = j3 + offset3;
    // solution(3) = j4 + offset4;
    // solution(4) = offset5 - j5;

    q_sols[0] = offset1 - j1;
    q_sols[1] = j2 + offset2;
    q_sols[2] = j3 + offset3;
    q_sols[3] = j4 + offset4;
    q_sols[4] = offset5 - j5;


    // std::stringstream sstr;
    // sstr << "Configuration without offsets: "
    //         << j1 << ", "
    //         << j2 << ", "
    //         << j3 << ", "
    //         << j4 << ", "
    //         << j5 << std::endl;
    // sstr << "Configuration with offsets: "
    //         << solution(0) << ", "
    //         << solution(1) << ", "
    //         << solution(2) << ", "
    //         << solution(3) << ", "
    //         << solution(4);
    // logger_.write(sstr.str(), __FILE__, __LINE__);

    return 1;
}


Transformation projectGoalOrientationIntoArmSubspace(
        const Transformation& goal)
{
    auto y_t_hat_m = goal.rotation().matrix();   // y vector of the rotation matrix
    auto z_t_hat_m = goal.rotation().matrix();   // z vector of the rotation matrix

    Vector3d y_t_hat(y_t_hat_m[0][1], y_t_hat_m[1][1], y_t_hat_m[2][1]);
    Vector3d z_t_hat(y_t_hat_m[0][2], y_t_hat_m[1][2], y_t_hat_m[2][2]);

    // m_hat is the normal of the "arm plane"
    Vector3d m_hat(0, -1, 0);

    // k_hat is the vector about which rotation of the goal frame is performed
    Vector3d k_hat = m_hat * z_t_hat;        // cross product

    // z_t_hat_tick is the new pointing direction of the arm
    KDL::Vector z_t_hat_tick = k_hat * m_hat;   // cross product

    // the amount of rotation
    double cos_theta = KDL::dot(z_t_hat, z_t_hat_tick);
    // first cross product then dot product
    double sin_theta = KDL::dot(z_t_hat * z_t_hat_tick, k_hat);

    // use Rodrigues' rotation formula to perform the rotation
    KDL::Vector y_t_hat_tick = (cos_theta * y_t_hat)
            // k_hat * y_t_hat is cross product
            + (sin_theta * (k_hat * y_t_hat)) + (1 - cos_theta)
            * (KDL::dot(k_hat, y_t_hat)) * k_hat;
    KDL::Vector x_t_hat_tick = y_t_hat_tick * z_t_hat_tick; // cross product

    KDL::Rotation rot(x_t_hat_tick, y_t_hat_tick, z_t_hat_tick);

    // the frame uses the old position but has the new, projected orientation
    return KDL::Frame(rot, goal.p);
}


bool InverseKinematics::isSolutionValid(const KDL::JntArray &solution) const
{
    bool valid = true;

    if (solution.rows() != 5) return false;

    for (unsigned int i = 0; i < solution.rows(); i++) {
        if ((solution(i) < min_angles_[i]) || (solution(i) > max_angles_[i])) {
            valid = false;
        }
    }

    return valid;
}



}
// #define IKFAST_HAS_LIBRARY
// #include "ikfast.h"
// using namespace ikfast;

// // check if the included ikfast version matches what this file was compiled with
// #define IKFAST_COMPILE_ASSERT(x) extern int __dummy[(int)x]
// IKFAST_COMPILE_ASSERT(IKFAST_VERSION==61);

// #ifdef IKFAST_NAMESPACE
// namespace IKFAST_NAMESPACE {
// #endif

// void to_mat44(double * mat4_4, const IkReal* eetrans, const IkReal* eerot)
// {
//     for(int i=0; i< 3;++i){
//         mat4_4[i*4+0] = eerot[i*3+0];
//         mat4_4[i*4+1] = eerot[i*3+1];
//         mat4_4[i*4+2] = eerot[i*3+2];
//         mat4_4[i*4+3] = eetrans[i];
//     }
//     mat4_4[3*4+0] = 0;
//     mat4_4[3*4+1] = 0;
//     mat4_4[3*4+2] = 0;
//     mat4_4[3*4+3] = 1;
// }

// void from_mat44(const double * mat4_4, IkReal* eetrans, IkReal* eerot)
// {
//     for(int i=0; i< 3;++i){
//         eerot[i*3+0] = mat4_4[i*4+0];
//         eerot[i*3+1] = mat4_4[i*4+1];
//         eerot[i*3+2] = mat4_4[i*4+2];
//         eetrans[i] = mat4_4[i*4+3];
//     }
// }


// IKFAST_API bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
//   if(!pfree) return false;

//   int n = GetNumJoints();
//   double q_sols[8*6];
//   double T[16];

//   to_mat44(T, eetrans, eerot);

//   int num_sols = ur_kinematics::inverse(T, q_sols,pfree[0]);

//   std::vector<int> vfree(0);

//   for (int i=0; i < num_sols; ++i){
//     std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(n);
//     for (int j=0; j < n; ++j) vinfos[j].foffset = q_sols[i*n+j];
//     solutions.AddSolution(vinfos,vfree);
//   }
//   return num_sols > 0;
// }

// IKFAST_API void ComputeFk(const IkReal* j, IkReal* eetrans, IkReal* eerot)
// {
//     double T[16];
//     ur_kinematics::forward(j,T);
//     from_mat44(T,eetrans,eerot);
// }

// IKFAST_API int GetNumFreeParameters() { return 1; }
// IKFAST_API int* GetFreeParameters() { static int freeparams[] = {5}; return freeparams; }
// IKFAST_API int GetNumJoints() { return 6; }

// IKFAST_API int GetIkRealSize() { return sizeof(IkReal); }

// #ifdef IKFAST_NAMESPACE
// } // end namespace
// #endif

// #ifndef IKFAST_NO_MAIN

// using namespace std;
// using namespace ur_kinematics;

// int main(int argc, char* argv[])
// {
//   double q[6] = {0.0, 0.0, 0.0, -1.571, -1.571, 0.0};
//   double* T = new double[16];
//   forward(q, T);
//   for(int i=0;i<4;i++) {
//     for(int j=i*4;j<(i+1)*4;j++)
//       printf("%1.3f ", T[j]);
//     printf("\n");
//   }
//   double q_sols[8*6];
//   int num_sols;
//   num_sols = inverse(T, q_sols);
//   for(int i=0;i<num_sols;i++) 
//     printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n", 
//        q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]);
//   for(int i=0;i<=4;i++)
//     printf("%f ", PI/2.0*i);
//   printf("\n");



//   printf("James's test\n");
//   // Point
//   T[3] = .7;
//   T[7] = 0;
//   T[11] = .2;

//   // Orientation matrix
//   T[0] = 0;
//   T[1] = 0;
//   T[2] = 1;

//   T[4] = 0;
//   T[5] = 1;
//   T[6] = 0;

//   T[8] = -1;
//   T[9] = 0;
//   T[10] = 0;

//   T[12] = 0;
//   T[13] = 0;
//   T[14] = 0;
//   T[15] = 1;
//   for(int i=0;i<4;i++) {
//     for(int j=i*4;j<(i+1)*4;j++)
//       printf("%1.3f ", T[j]);
//     printf("\n");
//   }
//   num_sols = inverse(T, q_sols);
//   for(int i=0;i<num_sols;i++) 
//     printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n", 
//        q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]);

//   return 0;
// }
// #endif
