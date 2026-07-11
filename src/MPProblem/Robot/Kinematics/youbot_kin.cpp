#include "youbot_kin.h"

#include <math.h>
#include <stdio.h>


namespace youbot_kinematics {
                                                        //     int inverse(const double* T, double* q_sols, double q6_des) {
                                                        //         #define DEG_TO_RAD(x) ((x) * PI / 180.0)

                                                        //         const double ALMOST_PLUS_ONE = 0.9999999;
                                                        //         const double ALMOST_MINUS_ONE = -0.9999999;


                                                        //         Vector3d translation = {T[3],T[7],T[11]};
                                                        //         std::cout << "IK translation: " << translation << std::endl;
                                                                
                                                        //         Matrix3x3 mat;
                                                        //         mat[0][0] = T[0];
                                                        //         mat[0][1] = T[1];
                                                        //         mat[0][2] = T[2];
                                                        //         mat[1][0] = T[4];
                                                        //         mat[1][1] = T[5];
                                                        //         mat[1][2] = T[6];
                                                        //         mat[2][0] = T[8];
                                                        //         mat[2][1] = T[9];
                                                        //         mat[2][2] = T[10];
                                                        //         Orientation rotation = mat;
                                                        //         Transformation g0(translation, rotation);

                                                        //         // KDL::Rotation R(T00,T01,T02,T10,T11,T12,T20,T21,T22);
                                                        //         // Vector3d V(T03,T13,T23);
                                                        //         // KDL::Frame g0(R, V); 	

                                                        //         bool offset_joint_1 = true;
                                                        //         bool offset_joint_3 = false;
                                                        //         // Parameters from youBot URDF file
                                                        //         double l0x = 0.024;
                                                        //         double l0z = 0.096;
                                                        //         double l1x = 0.033;
                                                        //         double l1z = 0.019;
                                                        //         double l2 = 0.155;
                                                        //         double l3 = 0.135;

                                                        //         // Distance from arm_link_4 to arm_link_5
                                                        //         double d = 0.13;

                                                        //         double j1 = 0.0;
                                                        //         double j2 = 0.0;
                                                        //         double j3 = 0.0;
                                                        //         double j4 = 0.0;
                                                        //         double j5 = 0.0;


                                                        //         // Transform from frame 0 to frame 1
                                                        //         Transformation frame0_to_frame1(Vector3d(-l0x,0.,-l0z),EulerAngle(0.,0.,0.));
                                                        //         Transformation g1 = frame0_to_frame1 * g0;

                                                        //         // First joint
                                                        //         j1 = atan2(g1.translation()[1], g1.translation()[0]);
                                                        //         std::cout << "xy is: " << g1.translation()[0] << " " << g1.translation()[1] << " -->" << j1*180/PI << std::endl;
                                                        //         if(offset_joint_1) {
                                                        //             if (j1 < 0.0) j1 += PI;
                                                        //             else j1 -= PI;
                                                        //         }


                                                        //         // Transform from frame 1 to frame 2
                                                        //         Transformation frame1_to_frame2(Vector3d(-l1x,0.,-l1z),EulerAngle(-j1,0,0));
                                                        //         Transformation g2 = frame1_to_frame2 * g1;

                                                        //         // Project the frame into the plane of the arm
                                                        //         auto g2_proj = youbot_kinematics::projectGoalOrientationIntoArmSubspace(g2);

                                                        //         // Set all values in the frame that are close to zero to exactly zero
                                                        //         for (int i = 0; i < 3; i++) {
                                                        //             for (int j = 0; j < 3; j++) {
                                                        //                 if (fabs(g2_proj.rotation().matrix()[i][j]) < 0.000001) g2_proj.rotation().matrix()[i][j] = 0;
                                                        //             }
                                                        //         }

                                                        //         // Fifth joint, determines the roll of the gripper (= wrist angle)
                                                        //         double s1 = sin(j1);
                                                        //         double c1 = cos(j1);
                                                        //         double r11 = g1.rotation().matrix()[0][0];
                                                        //         double r12 = g1.rotation().matrix()[0][1];
                                                        //         double r21 = g1.rotation().matrix()[1][0];
                                                        //         double r22 = g1.rotation().matrix()[1][1];
                                                        //         j5 = atan2(r21 * c1 - r11 * s1, r22 * c1 - r12 * s1);

                                                        //         // The sum of joint angles two to four determines the overall "pitch" of the
                                                        //         // end effector
                                                        //         double r13 = g2_proj.rotation().matrix()[0][2];
                                                        //         double r33 = g2_proj.rotation().matrix()[2][2];
                                                        //         double j234 = atan2(r13, r33);


                                                        //         Vector3d p2 = g2_proj.translation();

                                                        //         // In the arm's subplane, offset from the end-effector to the fourth joint
                                                        //         p2[0] = p2[0] - d * sin(j234);
                                                        //         p2[2] = p2[2] - d * cos(j234);


                                                        //         // Check if the goal position can be reached at all
                                                        //         if ((l2 + l3) < sqrt((p2[0] * p2[0]) + (p2[2] * p2[2]))) {
                                                        //             std::cout << "l2: " << l2 << std::endl;
                                                        //             std::cout << "l3: " << l3 << std::endl;
                                                        //             std::cout << "p2: " << p2 << std::endl;
                                                        //             std::cout << "dist: " << sqrt((p2[0] * p2[0]) + (p2[2] * p2[2])) << std::endl;
                                                                    
                                                        //             std::cout << "return 0" << std::endl;
                                                        //             return 0;
                                                        //         }

                                                        //         // Third joint
                                                        //         double l_sqr = (p2[0] * p2[0]) + (p2[2] * p2[2]);
                                                        //         double l2_sqr = l2 * l2;
                                                        //         double l3_sqr = l3 * l3;
                                                        //         double j3_cos = (l_sqr - l2_sqr - l3_sqr) / (2.0 * l2 * l3);

                                                        //         if (j3_cos > ALMOST_PLUS_ONE) j3 = 0.0;
                                                        //         else if (j3_cos < ALMOST_MINUS_ONE) j3 = PI;
                                                        //         else j3 = atan2(sqrt(1.0 - (j3_cos * j3_cos)), j3_cos);

                                                        //         if (offset_joint_3) j3 = -j3;


                                                        //         // Second joint
                                                        //         double t1 = atan2(p2[2], p2[0]);
                                                        //         double t2 = atan2(l3 * sin(j3), l2 + l3 * cos(j3));
                                                        //         j2 = PI/2. - t1 - t2;


                                                        //         // Fourth joint, determines the pitch of the gripper
                                                        //         j4 = j234 - j2 - j3;


                                                        //         // This IK assumes that the arm points upwards, so we need to consider
                                                        //         // the offsets to the real home position
                                                        //         // double offset1 = DEG_TO_RAD( 169.0);
                                                        //         // double offset2 = DEG_TO_RAD(  65.0);
                                                        //         // double offset3 = DEG_TO_RAD(-146.0);
                                                        //         // double offset4 = DEG_TO_RAD( 102.5);
                                                        //         // double offset5 = DEG_TO_RAD( 167.5);
                                                                
                                                        //         // double offset1 = DEG_TO_RAD( 0);
                                                        //         // double offset2 = DEG_TO_RAD( 0);
                                                        //         // double offset3 = DEG_TO_RAD( 0);
                                                        //         // double offset4 = DEG_TO_RAD( 0);
                                                        //         // double offset5 = DEG_TO_RAD( 0);

                                                        //         // q_sols[0] = offset1 - j1;
                                                        //         // q_sols[1] = j2 + offset2;
                                                        //         // q_sols[2] = j3 + offset3;
                                                        //         // q_sols[3] = j4 + offset4;
                                                        //         // q_sols[4] = offset5 - j5;

                                                        //         q_sols[0] = j1;
                                                        //         q_sols[1] = j2;
                                                        //         q_sols[2] = j3;
                                                        //         q_sols[3] = j4;
                                                        //         q_sols[4] = j5;


                                                        //         // std::stringstream sstr;
                                                        //         std::cout << "Configuration without offsets: "
                                                        //                 << j1/PI << ", "
                                                        //                 << j2/PI << ", "
                                                        //                 << j3/PI << ", "
                                                        //                 << j4/PI << ", "
                                                        //                 << j5/PI << std::endl;
                                                        //         std::cout << "Configuration with offsets: "
                                                        //                 << q_sols[0]/PI << ", "
                                                        //                 << q_sols[1]/PI << ", "
                                                        //                 << q_sols[2]/PI << ", "
                                                        //                 << q_sols[3]/PI << ", "
                                                        //                 << q_sols[4]/PI << std::endl;
                                                        //         // logger_.write(sstr.str(), __FILE__, __LINE__);

                                                        //         return 1;
                                                        //     }


                                                        //     Transformation projectGoalOrientationIntoArmSubspace(Transformation &goal)
                                                        //     {   
                                                        //         Vector3d y_t_hat = goal.rotation().getBasis(1);   // y vector of the rotation matrix
                                                        //         Vector3d z_t_hat = goal.rotation().getBasis(2);   // z vector of the rotation matrix
                                                                
                                                        //         double sum = 0;
                                                        //         for(auto e : y_t_hat) {
                                                        //             sum += fabs(e);
                                                        //         }
                                                        //         std::cout << "ythat: " << y_t_hat << std::endl;
                                                        //         y_t_hat /= sum;
                                                        //         std::cout << "ythat: " << y_t_hat << std::endl;


                                                        //         sum = 0;
                                                        //         for(auto e : z_t_hat) {
                                                        //             sum += fabs(e);
                                                        //         }
                                                        //         std::cout << "zthat: " << z_t_hat << std::endl;
                                                        //         z_t_hat /= sum;
                                                        //         std::cout << "zthat: " << z_t_hat << std::endl;


                                                        //         // m_hat is the normal of the "arm plane"
                                                        //         Vector3d m_hat = {0, -1, 0};

                                                        //         // k_hat is the vector about which rotation of the goal frame is performed
                                                        //         Vector3d k_hat = m_hat % z_t_hat;        // cross product

                                                        //         // z_t_hat_tick is the new pointing direction of the arm
                                                        //         Vector3d z_t_hat_tick = k_hat % m_hat;   // cross product

                                                        //         // the amount of rotation
                                                        //         double cos_theta = z_t_hat * z_t_hat_tick;
                                                        //         // first cross product then dot product
                                                        //         double sin_theta = (z_t_hat % z_t_hat_tick) * k_hat;

                                                        //         // use Rodrigues' rotation formula to perform the rotation
                                                        //         Vector3d y_t_hat_tick = (cos_theta * y_t_hat)
                                                        //                 // k_hat * y_t_hat is cross product
                                                        //                 + (sin_theta * (k_hat % y_t_hat)) + (1 - cos_theta)
                                                        //                 * (k_hat * y_t_hat) * k_hat;
                                                        //         Vector3d x_t_hat_tick = y_t_hat_tick % z_t_hat_tick; // cross product
                                                                
                                                        //         Matrix3x3 mat;
                                                        //         mat[0][0] = x_t_hat_tick[0];
                                                        //         mat[1][0] = x_t_hat_tick[1];
                                                        //         mat[2][0] = x_t_hat_tick[2];
                                                                
                                                        //         mat[0][1] = y_t_hat_tick[0];
                                                        //         mat[1][1] = y_t_hat_tick[1];
                                                        //         mat[2][1] = y_t_hat_tick[2];

                                                        //         mat[0][2] = z_t_hat_tick[0];
                                                        //         mat[1][2] = z_t_hat_tick[1];
                                                        //         mat[2][2] = z_t_hat_tick[2];

                                                        //         Orientation rot(mat);

                                                        //         // the frame uses the old position but has the new, projected orientation
                                                        //         return Transformation(goal.translation(), rot);
                                                        //     }

                                                        // };



    const std::vector<double> dh_a = {0.033, 0.155, 0.135, 0, 0};
    const std::vector<double> dh_alpha = {PI/2, 0, 0, PI/2, 0};
    const std::vector<double> dh_d = {0.147, 0, 0, 0, 0.218};
    // DH_THETA = (np.pi*169.0/180.0, np.pi*65.0/180.0+np.pi/2, -np.pi*146.0/180.0, np.pi*102.5/180.0+np.pi/2, np.pi*167.5/180.0)  #theta = DH_THETA - q
    const std::vector<double> dh_theta = {0., 0., 0., 0., 0.};  // theta = dh_theta - q


    // int inverse(const double* T, double* q_sols, double q6_des) {

    //     Vector3d translation = {T[3],T[7],T[11]};
    //     std::cout << "IK translation: " << translation << std::endl;

    //     Matrix3x3 mat;
    //     mat[0][0] = T[0];
    //     mat[0][1] = T[1];
    //     mat[0][2] = T[2];
    //     mat[1][0] = T[4];
    //     mat[1][1] = T[5];
    //     mat[1][2] = T[6];
    //     mat[2][0] = T[8];
    //     mat[2][1] = T[9];
    //     mat[2][2] = T[10];
    //     Orientation rotation = mat;
    //     std::cout << "IK rotation: " << rotation << std::endl;
    //     std::cout << "IK rotation: " << rotation.matrix() << std::endl;
    //     for(size_t i ; i < 3 ; i++) {
    //         for(size_t j ; j < 3 ; j++) {
    //             std::cout << mat[i][j] << " " ;
    //         }
    //         std::cout << std::endl;
    //     }

    //     Transformation g0(translation, rotation);
        
    //     double px = translation[0];
    //     double py = translation[1];
    //     // double pz = translation[2];
    
    //     // theta_1
    //     double theta_1_I = atan2(py, px);
    //     double theta_1_II = atan2(-py, -px);

    //     //theta_2
    //     double s5 = g0.rotation().matrix()[0][0] * sin(theta_1_I) - g0.rotation().matrix()[1][0] * cos(theta_1_I);
    //     double c5 = g0.rotation().matrix()[0][1] * sin(theta_1_I) - g0.rotation().matrix()[1][1] * cos(theta_1_I);
    //     double theta_5_I = atan2(s5,c5);
    //     s5 = g0.rotation().matrix()[0][0] * sin(theta_1_II) - g0.rotation().matrix()[1][0] * cos(theta_1_II);
    //     c5 = g0.rotation().matrix()[0][1] * sin(theta_1_II) - g0.rotation().matrix()[1][1] * cos(theta_1_II);
    //     double theta_5_II = atan2(s5,c5);

    //     // theta_234
    //     double c234 = -g0.rotation().matrix()[2][2];
    //     double s234 = g0.rotation().matrix()[0][2] * cos(theta_1_I) + g0.rotation().matrix()[1][2] * sin(theta_1_I);
    //     double theta_234_I = atan2(s234,c234);
    //     s234 = g0.rotation().matrix()[0][2] * cos(theta_1_II) + g0.rotation().matrix()[1][2] * sin(theta_1_II);
    //     double theta_234_II = atan2(s234, c234);

    //     // theta_3
    //     Vector3d v = get_vector_1to4_frame(theta_1_I, g0.rotation().matrix(), translation);
    //     double xr = v[0], yr = v[1], zr = v[2];
    //     double numerator_0 = xr * xr + yr * yr + zr * zr - dh_a[1] * dh_a[1] - dh_a[2] * dh_a[2];
    //     double denominator_0 = 2 * dh_a[1] * dh_a[2];
    //     double cosq3_I = numerator_0 / denominator_0;

    //     if (cosq3_I <= 1) {
    //         double theta_3_I = -atan2(sqrt(1.0 - cosq3_I * cosq3_I), cosq3_I);
    //         double theta_3_II = atan2(sqrt(1.0 - cosq3_I * cosq3_I), cosq3_I);

    //         double phi_I = atan2(yr, xr);
    //         double beta_I = atan2(dh_a[2] * sin(abs(theta_3_I)), dh_a[1] + dh_a[2] * cos(abs(theta_3_I)));
    //         double beta_II = atan2(dh_a[2] * sin(abs(theta_3_II)), dh_a[1] + dh_a[2] * cos(abs(theta_3_II)));

    //         double theta_2_I = phi_I + beta_I;
    //         double theta_2_II = phi_I - beta_II;

    //         double theta_4_I = theta_234_I - theta_2_I - theta_3_I;
    //         double theta_4_II = theta_234_I - theta_2_II - theta_3_II;
            
    //         q_sols[0] = theta_1_I;
    //         q_sols[1] = theta_2_I;
    //         q_sols[2] = theta_3_I;
    //         q_sols[3] = theta_4_I;
    //         q_sols[4] = theta_5_I;

    //         q_sols[5] = theta_1_I;
    //         q_sols[6] = theta_2_II;
    //         q_sols[7] = theta_3_II;
    //         q_sols[8] = theta_4_II;
    //         q_sols[9] = theta_5_I;
    //     } else {
    //         std::cout << "No solutions for q_I and q_II" << std::endl;
    //     }

    //     v = get_vector_1to4_frame(theta_1_II, g0.rotation().matrix(), translation);
    //     xr = v[0]; yr = v[1]; zr = v[2];
    //     xr -= FLT_EPSILON;
    //     numerator_0 = xr * xr + yr * yr + zr * zr - dh_a[1]*dh_a[1] - dh_a[2]*dh_a[2];
    //     denominator_0 = 2 * dh_a[1] * dh_a[2];
    //     double cosq3_II = numerator_0 / denominator_0  - copysign(1.0, numerator_0) * FLT_EPSILON;

    //     if (cosq3_II <= 1) {
    //         double theta_3_III = -atan2(sqrt(1.0 - cosq3_II * cosq3_II), cosq3_II);
    //         double theta_3_IV = atan2(sqrt(1.0 - cosq3_II * cosq3_II), cosq3_II);

    //         double phi_II = atan2(yr, xr);
    //         double beta_III = atan2(dh_a[2] * sin(abs(theta_3_III)), dh_a[1] + dh_a[2] * cos(abs(theta_3_III)));
    //         double beta_IV = atan2(dh_a[2] * sin(abs(theta_3_IV)), dh_a[1] + dh_a[2] * cos(abs(theta_3_IV)));

    //         double theta_2_III = phi_II + beta_III;
    //         double theta_2_IV = phi_II - beta_IV;

    //         double theta_4_III = theta_234_II - theta_2_III - theta_3_III;
    //         double theta_4_IV = theta_234_II - theta_2_IV - theta_3_IV;

    //         q_sols[10] = theta_1_II;
    //         q_sols[11] = theta_2_III;
    //         q_sols[12] = theta_3_III;
    //         q_sols[13] = theta_4_III;
    //         q_sols[14] = theta_5_II;

    //         q_sols[15] = theta_1_II;
    //         q_sols[16] = theta_2_IV;
    //         q_sols[17] = theta_3_IV;
    //         q_sols[18] = theta_4_IV;
    //         q_sols[19] = theta_5_II;

    //     } else {
    //         std::cout << "No solutions for q_III and q_IV" << std::endl;
    //     }

    //     for(int i = 0; i < 4; i++) {
    //         for (int j = 0; j < 5; j++) {
                
    //             q_sols[i*5+j] = 0 - q_sols[i*5+j]; // Offsets
    //             q_sols[i*5+j] = normalized_angle(q_sols[i*5+j], j);

    //         }
    //     }

    //     return 4;
    // }











    int inverse(const double* T, double* q_sols, double q6_des) {
        int num_sol = 0;
        double l0 = 0.147;
        double l1 = 0.155;
        double l2 = 0.135;
        double l3 = 0.2174;

        // Vector3d translation = {fabs(T[3]),T[7],T[11]};
        Vector3d translation = {T[11], T[7], fabs(T[3])}; 
        // Vector3d translation = {0.2, T[7], 0.2}; 
        // X: 0.2 --> 0.15  ==>  lower in the Z axis   ( v )
        // Z: 0.2 --> 0.15  ==>  far in the X axis ( < )

        std::cout << "IK translation: " << translation << std::endl;

        Matrix3x3 mat;
        mat[0][0] = T[0];
        mat[0][1] = T[1];
        mat[0][2] = T[2];
        mat[1][0] = T[4];
        mat[1][1] = T[5];
        mat[1][2] = T[6];
        mat[2][0] = T[8];
        mat[2][1] = T[9];
        mat[2][2] = T[10];

        // mat[0][0] = 1;
        // mat[0][1] = 0;
        // mat[0][2] = 0;
        // mat[1][0] = 0;
        // mat[1][1] = -1;
        // mat[1][2] = 0;
        // mat[2][0] = 0;
        // mat[2][1] = 0;
        // mat[2][2] = -1;

        Orientation rotation = mat;
        std::cout << "IK rotation: " << rotation << std::endl;
        std::cout << "IK rotation: " << rotation.matrix() << std::endl;
        for(size_t i = 0 ; i < 3 ; i++) {
            for(size_t j ; j < 3 ; j++) {
                std::cout << mat[i][j] << " " ;
            }
            std::cout << std::endl;
        }

        Transformation g0(translation, rotation);
        
        double px = -T[3];
        double py = T[7];
        // double pz = translation[2];
    





        double theta1_I = atan2(py, px);
        
        if(fabs(theta1_I) < 169.*PI/180.) {
            std::cout << "theta1_I: " << theta1_I*180/PI << std::endl;
            double s5_I = g0.rotation().matrix()[0][0] * sin(theta1_I) - g0.rotation().matrix()[0][1] * cos(theta1_I);
            double c5_I = g0.rotation().matrix()[1][0] * sin(theta1_I) - g0.rotation().matrix()[1][1] * cos(theta1_I);  

            if(fabs(s5_I) <= 1 and fabs(c5_I) <= 1) {

                double theta5_I = atan2(s5_I,c5_I);
                std::cout << "theta5_I: " << theta5_I*180/PI << std::endl;
                
                if(fabs(theta5_I) < 167.5*PI/180.) {

                    double c234_I = g0.rotation().matrix()[2][2];
                    double s234_I = - g0.rotation().matrix()[1][2] * sin(theta1_I) - g0.rotation().matrix()[0][2] * cos(theta1_I);
                    
                    if(fabs(c234_I) <= 1 and fabs(s234_I) <= 1) {

                        double theta234_I = atan2(s234_I,c234_I);

                        std::cout << "check zero" << std::endl;
                        double criterion1 = - g0.rotation().matrix()[0][0]*cos(theta1_I)*s234_I - g0.rotation().matrix()[1][0]*sin(theta1_I)*s234_I + g0.rotation().matrix()[2][0]*c234_I;
                        double criterion2 = - g0.rotation().matrix()[0][1]*cos(theta1_I)*s234_I - g0.rotation().matrix()[1][1]*sin(theta1_I)*s234_I + g0.rotation().matrix()[2][1]*c234_I;

                        if(criterion1 < 0.000001 and criterion2 < 0.000001) {
                            std::cout << "criterions: " << criterion1 << " " << criterion2 << std::endl;

                            double dx_I = g0.translation()[0] / cos(theta1_I) - l3*s234_I;
                            double dz_I = g0.translation()[2] - l0 + l3*c234_I;
                            // double dx_I = g0.translation()[0] / cos(theta1_I) - l3*c234_I;
                            // double dz_I = g0.translation()[2] - l0 - l3*s234_I;

                            // double theta3_I_I = acos((l1*l1 + l2*l2 - dx_I*dx_I - dz_I*dz_I)/(2*l1*l2));
                            double theta3_I_I = acos(-(l1*l1 + l2*l2 - dx_I*dx_I - dz_I*dz_I)/(2*l1*l2));

                            std::cout << "theta3_I_I: " << theta3_I_I << std::endl;
                            std::cout <<  theta1_I << std::endl;
                            std::cout <<  dx_I << " " << dz_I << std::endl;
                            std::cout << (l1*l1 + l2*l2 - dx_I*dx_I - dz_I*dz_I)/(2 *l1*l2) << std::endl;
                            std::cout <<  l1*l1 << " " << l2*l2 << " " << dx_I*dx_I << " " << dz_I*dz_I << std::endl;
                            if(fabs(theta3_I_I) < 146.*PI/180.) {


                                if(((l1-l2)*(l1-l2) <= dx_I*dx_I + dz_I*dz_I) and (dx_I*dx_I + dz_I*dz_I <= (l1+l2)*(l1+l2))){
                                    double theta2_I_I;
                                    double theta4_I_I;
                                    double phi;
                                    double beta;
                                    if(theta3_I_I < 0) {
                                        phi = atan2(dz_I,dx_I);
                                        beta = acos((dx_I*dx_I + dz_I*dz_I + l1*l1 - l2*l2)/(2*l1*sqrt(dx_I*dx_I + dz_I*dz_I)));
                                        theta2_I_I = phi + beta;
                                        theta4_I_I = theta234_I - theta2_I_I - theta3_I_I;
                                        std::cout << "< 0 : " << theta2_I_I*180/PI << " " << theta4_I_I*180/PI << std::endl;
                                    }
                                    else {
                                        phi = atan2(dz_I,dx_I);
                                        beta = acos((dx_I*dx_I + dz_I*dz_I + l1*l1 - l2*l2)/(2*l1*sqrt(dx_I*dx_I + dz_I*dz_I)));
                                        theta2_I_I = phi - beta;        
                                        theta4_I_I = theta234_I - theta2_I_I - theta3_I_I;
                                        std::cout << "> 0 : " << theta2_I_I*180/PI << " " << theta4_I_I*180/PI << std::endl;
                                    }

                                    if(fabs(theta4_I_I) < 102.5*PI/180. and theta2_I_I < 90.*PI/180. and theta2_I_I > -65.*PI/180.) {
                                    // if(1) {
                                        q_sols[num_sol+0] = theta1_I;
                                        q_sols[num_sol+1] = theta2_I_I;
                                        q_sols[num_sol+2] = theta3_I_I;
                                        q_sols[num_sol+3] = theta4_I_I;
                                        q_sols[num_sol+4] = theta5_I;
                                        num_sol++;
                                    }
                                    else {
                                        std::cout << "theta4_I_I or theta2_I_I out of range" << std::endl;
                                    }
                                }
                                else {
                                    std::cout << "dx_I out of range" << std::endl;
                                }
                            }
                            else {
                                std::cout << "theta3_I_I out of range" << std::endl;
                            }
                        

                            double theta3_I_II = -acos(-(l1*l1 + l2*l2 - dx_I*dx_I - dz_I*dz_I)/(2*l1*l2));
                            // double theta3_I_II = -acos((l1*l1 + l2*l2 - dx_I*dx_I - dz_I*dz_I)/(2*l1*l2));

                            std::cout << "theta3_I_II: " << theta3_I_II << std::endl;

                            if(fabs(theta3_I_II) < 146.*PI/180.) {
                                
                                if(((l1-l2)*(l1-l2) <= dx_I*dx_I + dz_I*dz_I) and (dx_I*dx_I + dz_I*dz_I <= (l1+l2)*(l1+l2))){
                                    double theta2_I_II;
                                    double theta4_I_II;
                                    double phi;
                                    double beta;
                                    if(theta3_I_II < 0) {
                                        phi = atan2(dz_I,dx_I);
                                        beta = acos((dx_I*dx_I + dz_I*dz_I + l1*l1 - l2*l2)/(2*l1*sqrt(dx_I*dx_I + dz_I*dz_I)));
                                        theta2_I_II = phi + beta;
                                        theta4_I_II = theta234_I - theta2_I_II - theta3_I_II;
                                    }
                                    else {
                                        phi = atan2(dz_I,dx_I);
                                        beta = acos((dx_I*dx_I + dz_I*dz_I + l1*l1 - l2*l2)/(2*l1*sqrt(dx_I*dx_I + dz_I*dz_I)));
                                        theta2_I_II = phi - beta;        
                                        theta4_I_II = theta234_I - theta2_I_II - theta3_I_II;
                                    }
                                    // if(fabs(theta4_I_II) < 102.5*PI/180. and theta2_I_II < 90.*PI/180. and theta2_I_II > -65.*PI/180.) {
                                    if(1) {
                                        q_sols[num_sol+0] = theta1_I;
                                        q_sols[num_sol+1] = theta2_I_II;
                                        q_sols[num_sol+2] = theta3_I_II;
                                        q_sols[num_sol+3] = theta4_I_II;
                                        q_sols[num_sol+4] = theta5_I;
                                        num_sol++;
                                    }
                                    else {
                                        std::cout << "theta4_I_II or theta2_I_II out of range" << std::endl;
                                    }
                                }
                                else {
                                    std::cout << "dx_I out of range" << std::endl;
                                }
                            }
                            else {
                                std::cout << "theta3_I_II out of range" << std::endl;
                            }
                        }
                        else {
                            std::cout << "criteria not satisfied" << std::endl;
                        }
                    }
                    else {
                        std::cout << "c234_I or s234_I out of range" << std::endl;
                    }
                }
                else {
                    std::cout << "theta5_I out of range" << std::endl;
                }
            }
            else {
                std::cout << "s5_I or c5_I out of range" << std::endl;
            }
        }
        else {
            std::cout << "theta1_I out of range" << std::endl;
        }

        double theta1_II = atan2(-py, -px);
        std::cout << "theta1_II: " << theta1_II << std::endl;

        if(fabs(theta1_II) < 169.*PI/180.) {
            std::cout << "theta1_II: " << theta1_II*180/PI << std::endl;
            double s5_II = g0.rotation().matrix()[0][0] * sin(theta1_II) - g0.rotation().matrix()[0][1] * cos(theta1_II);
            double c5_II = g0.rotation().matrix()[1][0] * sin(theta1_II) - g0.rotation().matrix()[1][1] * cos(theta1_II);  

            if(fabs(s5_II) <= 1 and fabs(c5_II) <= 1) {

                double theta5_II = atan2(s5_II,c5_II);
                std::cout << "theta5_II: " << theta5_II*180/PI << std::endl;
                
                if(fabs(theta5_II) < 167.5*PI/180.) {

                    double c234_II = g0.rotation().matrix()[2][2];
                    double s234_II = - g0.rotation().matrix()[1][2] * sin(theta1_II) - g0.rotation().matrix()[0][2] * cos(theta1_II);
                    
                    if(fabs(c234_II) <= 1 and fabs(s234_II) <= 1) {

                        double theta234_II = atan2(s234_II,c234_II);

                        std::cout << "check zero" << std::endl;
                        double criterion1 = - g0.rotation().matrix()[0][0]*cos(theta1_II)*s234_II - g0.rotation().matrix()[1][0]*sin(theta1_II)*s234_II + g0.rotation().matrix()[2][0]*c234_II;
                        double criterion2 = - g0.rotation().matrix()[0][1]*cos(theta1_II)*s234_II - g0.rotation().matrix()[1][1]*sin(theta1_II)*s234_II + g0.rotation().matrix()[2][1]*c234_II;

                        if(criterion1 < 0.000001 and criterion2 < 0.000001) {
                            std::cout << "criterions: " << criterion1 << " " << criterion2 << std::endl;

                            double dx_II = g0.translation()[0] / cos(theta1_II) - l3*s234_II;
                            double dz_II = g0.translation()[2] - l0 + l3*c234_II;
                            // double dx_II = g0.translation()[0] / cos(theta1_II) - l3*c234_II;
                            // double dz_II = g0.translation()[2] - l0 - l3*s234_II;

                            double theta3_II_I = acos(-(l1*l1 + l2*l2 - dx_II*dx_II - dz_II*dz_II)/(2*l1*l2));
                            // double theta3_II_I = acos((l1*l1 + l2*l2 - dx_II*dx_II - dz_II*dz_II)/(2*l1*l2));
                            
                            std::cout << "theta3_II_I: " << theta3_II_I << std::endl;
                            std::cout <<  theta1_II << std::endl;
                            std::cout <<  dx_II << " " << dz_II << std::endl;
                            std::cout << (l1*l1 + l2*l2 - dx_II*dx_II - dz_II*dz_II)/(2 *l1*l2) << std::endl;
                            std::cout <<  l1*l1 << " " << l2*l2 << " " << dx_II*dx_II << " " << dz_II*dz_II << std::endl;
                    
                            if(fabs(theta3_II_I) < 146.*PI/180.) {


                                if(((l1-l2)*(l1-l2) <= dx_II*dx_II + dz_II*dz_II) and (dx_II*dx_II + dz_II*dz_II <= (l1+l2)*(l1+l2))){
                                    double theta2_II_I;
                                    double theta4_II_I;
                                    double phi;
                                    double beta;
                                    if(theta3_II_I < 0) {
                                        phi = atan2(dz_II,dx_II);
                                        beta = acos((dx_II*dx_II + dz_II*dz_II + l1*l1 - l2*l2)/(2*l1*sqrt(dx_II*dx_II + dz_II*dz_II)));
                                        theta2_II_I = phi + beta;
                                        theta4_II_I = theta234_II - theta2_II_I - theta3_II_I;
                                        std::cout << "< 0 : " << theta2_II_I*180/PI << " " << theta4_II_I*180/PI << std::endl;
                                    }
                                    else {
                                        phi = atan2(dz_II,dx_II);
                                        beta = acos((dx_II*dx_II + dz_II*dz_II + l1*l1 - l2*l2)/(2*l1*sqrt(dx_II*dx_II + dz_II*dz_II)));
                                        theta2_II_I = phi - beta;        
                                        theta4_II_I = theta234_II - theta2_II_I - theta3_II_I;
                                        std::cout << "> 0 : " << theta2_II_I*180/PI << " " << theta4_II_I*180/PI << std::endl;
                                    }

                                    if(fabs(theta4_II_I) < 102.5*PI/180. and theta2_II_I < 90.*PI/180. and theta2_II_I > -65.*PI/180.) {
                                    // if(1) {
                                        q_sols[num_sol+0] = theta1_II;
                                        q_sols[num_sol+1] = theta2_II_I;
                                        q_sols[num_sol+2] = theta3_II_I;
                                        q_sols[num_sol+3] = theta4_II_I;
                                        q_sols[num_sol+4] = theta5_II;
                                        num_sol++;
                                    }
                                    else {
                                        std::cout << "theta4_II_I or theta2_II_I out of range" << std::endl;
                                    }
                                }
                                else {
                                    std::cout << "dx_II out of range" << std::endl;
                                }
                            }
                            else {
                                std::cout << "theta3_II_I out of range" << std::endl;
                            }
                        

                            double theta3_II_II = -acos(-(l1*l1 + l2*l2 - dx_II*dx_II - dz_II*dz_II)/(2*l1*l2));
                            // double theta3_II_II = -acos((l1*l1 + l2*l2 - dx_II*dx_II - dz_II*dz_II)/(2*l1*l2));

                            std::cout << "theta3_II_II: " << theta3_II_II << std::endl;

                            if(fabs(theta3_II_II) < 146.*PI/180.) {
                                
                                if(((l1-l2)*(l1-l2) <= dx_II*dx_II + dz_II*dz_II) and (dx_II*dx_II + dz_II*dz_II <= (l1+l2)*(l1+l2))){
                                    double theta2_II_II;
                                    double theta4_II_II;
                                    double phi;
                                    double beta;
                                    if(theta3_II_II < 0) {
                                        phi = atan2(dz_II,dx_II);
                                        beta = acos((dx_II*dx_II + dz_II*dz_II + l1*l1 - l2*l2)/(2*l1*sqrt(dx_II*dx_II + dz_II*dz_II)));
                                        theta2_II_II = phi + beta;
                                        theta4_II_II = theta234_II - theta2_II_II - theta3_II_II;
                                    }
                                    else {
                                        phi = atan2(dz_II,dx_II);
                                        beta = acos((dx_II*dx_II + dz_II*dz_II + l1*l1 - l2*l2)/(2*l1*sqrt(dx_II*dx_II + dz_II*dz_II)));
                                        theta2_II_II = phi - beta;        
                                        theta4_II_II = theta234_II - theta2_II_II - theta3_II_II;
                                    }
                                    if(fabs(theta4_II_II) < 102.5*PI/180. and theta2_II_II < 90.*PI/180. and theta2_II_II > -65.*PI/180.) {
                                        q_sols[num_sol+0] = theta1_II;
                                        q_sols[num_sol+1] = theta2_II_II;
                                        q_sols[num_sol+2] = theta3_II_II;
                                        q_sols[num_sol+3] = theta4_II_II;
                                        q_sols[num_sol+4] = theta5_II;
                                        num_sol++;
                                    }
                                    else {
                                        std::cout << "theta4_II_II or theta2_II_II out of range" << std::endl;
                                    }
                                }
                                else {
                                    std::cout << "dx_II out of range" << std::endl;
                                }
                            }
                            else {
                                std::cout << "theta3_II_II out of range" << std::endl;
                            }
                        }
                        else {
                            std::cout << "criteria not satisfied" << std::endl;
                        }
                    }
                    else {
                        std::cout << "c234_II or s234_II out of range" << std::endl;
                    }
                }
                else {
                    std::cout << "theta5_II out of range" << std::endl;
                }
            }
            else {
                std::cout << "s5_II or c5_II out of range" << std::endl;
            }
        }
        else {
            std::cout << "theta1_II out of range" << std::endl;
        }
        
        std::cout << "The Number of solutions: " << num_sol << std::endl;
        return num_sol;
    }




    Vector3d get_vector_1to4_frame(double theta, Matrix3x3 pr, Vector3d p) {

        Vector3d v;
        double px = p[0], py = p[1], pz = p[2];
        double c1 = cos(theta);
        double s1 = sin(theta);
        double xr = (py - dh_d[4] * pr[1][2]) * s1 + (px - dh_d[4] * pr[0][2]) * c1 - dh_a[0];
        double yr = pz - dh_d[0] - dh_d[4] * pr[2][2];
        double zr = 0.0;
        v[0] = xr;
        v[1] = yr;
        v[2] = zr;

        return v;
    }


    double normalized_angle(double q, int j) {
        if (j != 2) {
            while (q < 0.0) {
                q += 2. * M_PI;
            }
            while (q >= 2 * M_PI) {
                q -= 2. * M_PI;
            }
        } else {
            while (q > 0.0) {
                q -= 2. * M_PI;
            }
            while (q <= -2 * M_PI) {
                q += 2. * M_PI;
            }
        }
        return q;
    }
};