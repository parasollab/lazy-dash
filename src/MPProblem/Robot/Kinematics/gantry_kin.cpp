#include "gantry_kin.h"

#include <math.h>
#include <stdio.h>


namespace gantry_kinematics {
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

  void inverse(const double* T, double* q_sols, double q6_des) {
    double T02 = *T; 
    T++; 
    // double T00 =  *T; 
    T++; 
    // double T01 =  *T; 
    T++; 
    double T03 = *T; 
    T++; 
    double T12 = *T; 
    T++; 
    // double T10 =  *T; 
    T++; 
    // double T11 =  *T; 
    T++; 
    double T13 = *T; 
    T++; 
    // double T22 =  *T; 
    T++; 
    // double T20 = -*T; 
    T++; 
    // double T21 = -*T; 
    T++; 
    double T23 =  *T;

    q_sols[0] = atan(T12/T02); 
    q_sols[1] = T03; 
    q_sols[2] = T13; 
    q_sols[3] = T23; 
  }
};


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
