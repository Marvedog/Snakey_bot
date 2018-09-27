#include "pkg_utils/transformations.h"
#include <gtest/gtest.h>

#include <math.h>

TEST(TransformationsTest, Rot_x) 
{
  
	double TOL = 0.00001;

 	/* No rotation */	
	Eigen::MatrixXd rot_1(3,3);
	rot_1 << 1, 0, 0,
	    		 0, 1, 0, 
				   0, 0, 1;

	/* Rotation pi/4 */
	Eigen::MatrixXd rot_2(3,3);
	rot_2 << 1, 			 0, 				0,
					 0, 0.707107, -0.707107, 
				   0, 0.707107,  0.707107;	 
 	
	/* Rotation pi/2 */	
	Eigen::MatrixXd rot_3(3,3);
	rot_3 << 1, 0,  0,
					 0, 0, -1, 
				   0, 1,  0;	 
	/* Rotation 3pi/4 */
	Eigen::MatrixXd rot_4(3,3);
	rot_4 << 1, 				0,          0,
	  			 0, -0.707107,  -0.707107, 
				   0,  0.707107,  -0.707107;	 
 	/* Rotation pi */	
	Eigen::MatrixXd rot_5(3,3);
	rot_5 << 1,  0,  0,
					 0, -1,  0, 
				   0,  0, -1;	 
	
	/* Rotations */
	double x_1 = 0;
	double x_2 = M_PI/4;
	double x_3 = M_PI/2;
	double x_4 = 3*M_PI/4;
	double x_5 = M_PI;

	/* Pre-allocated matrices */
	Eigen::Matrix3d out_1(3,3);
	Eigen::Matrix3d out_2(3,3);
	Eigen::Matrix3d out_3(3,3);
	Eigen::Matrix3d out_4(3,3);
	Eigen::Matrix3d out_5(3,3);

	utils::rot_x(x_1, out_1);
	utils::rot_x(x_2, out_2);
	utils::rot_x(x_3, out_3);
	utils::rot_x(x_4, out_4);
	utils::rot_x(x_5, out_5);
	
	ASSERT_TRUE(out_1.isApprox(rot_1, TOL));
  ASSERT_TRUE(out_2.isApprox(rot_2, TOL));
	ASSERT_TRUE(out_3.isApprox(rot_3, TOL));
	ASSERT_TRUE(out_4.isApprox(rot_4, TOL));
	ASSERT_TRUE(out_5.isApprox(rot_5, TOL));
}

TEST(TransformationsTest, Rot_y) 
{
  
	double TOL = 0.00001;

 	/* No rotation */	
	Eigen::MatrixXd rot_1(3,3);
	rot_1 << 1, 0, 0,
	    		 0, 1, 0, 
				   0, 0, 1;

	/* Rotation pi/4 */
	Eigen::MatrixXd rot_2(3,3);
	rot_2 <<  0.707107, 0,  0.707107,
	  			 				0,  1, 				 0, 
				   -0.707107, 0,  0.707107;	 
 	
	/* Rotation pi/2 */	
	Eigen::MatrixXd rot_3(3,3);
	rot_3 <<  0, 0, 1,
					  0, 1, 0, 
				   -1, 0, 0;	 
	/* Rotation 3pi/4 */
	Eigen::MatrixXd rot_4(3,3);
	rot_4 << -0.707107, 0,  0.707107,
	  			 				0,  1, 				 0, 
				   -0.707107, 0, -0.707107;	 
 	/* Rotation pi */	
	Eigen::MatrixXd rot_5(3,3);
	rot_5 << -1,  0,  0,
					  0,  1,  0, 
				    0,  0, -1;	 
	
	/* Rotations */
	double y_1 = 0;
	double y_2 = M_PI/4;
	double y_3 = M_PI/2;
	double y_4 = 3*M_PI/4;
	double y_5 = M_PI;

	/* Pre-allocated matrices */
	Eigen::Matrix3d out_1(3,3);
	Eigen::Matrix3d out_2(3,3);
	Eigen::Matrix3d out_3(3,3);
	Eigen::Matrix3d out_4(3,3);
	Eigen::Matrix3d out_5(3,3);

	utils::rot_y(y_1, out_1);
	utils::rot_y(y_2, out_2);
	utils::rot_y(y_3, out_3);
	utils::rot_y(y_4, out_4);
	utils::rot_y(y_5, out_5);
	
	ASSERT_TRUE(out_1.isApprox(rot_1, TOL));
  ASSERT_TRUE(out_2.isApprox(rot_2, TOL));
	ASSERT_TRUE(out_3.isApprox(rot_3, TOL));
	ASSERT_TRUE(out_4.isApprox(rot_4, TOL));
	ASSERT_TRUE(out_5.isApprox(rot_5, TOL));
}


TEST(TransformationsTest, Rot_z) 
{
  
	double TOL = 0.00001;

 	/* No rotation */	
	Eigen::MatrixXd rot_1(3,3);
	rot_1 << 1, 0, 0,
	    		 0, 1, 0, 
				   0, 0, 1;

	/* Rotation pi/4 */
	Eigen::MatrixXd rot_2(3,3);
	rot_2 << 0.707107, -0.707107, 0,
					 0.707107,  0.707107, 0, 
				   				0, 			   0, 1;	 
 	
	/* Rotation pi/2 */	
	Eigen::MatrixXd rot_3(3,3);
	rot_3 << 0, -1, 0,
					 1,  0, 0, 
				   0,  0, 1;	 
	/* Rotation 3pi/4 */
	Eigen::MatrixXd rot_4(3,3);
	rot_4 << -0.707107, -0.707107,  0,
	  			  0.707107, -0.707107,  0, 
				   				 0,         0,  1;
 	/* Rotation pi */	
	Eigen::MatrixXd rot_5(3,3);
	rot_5 << -1,  0,  0,
					  0, -1,  0, 
				    0,  0,  1;	 
	
	/* Rotations */
	double z_1 = 0;
	double z_2 = M_PI/4;
	double z_3 = M_PI/2;
	double z_4 = 3*M_PI/4;
	double z_5 = M_PI;

	/* Pre-allocated matrices */
	Eigen::Matrix3d out_1(3,3);
	Eigen::Matrix3d out_2(3,3);
	Eigen::Matrix3d out_3(3,3);
	Eigen::Matrix3d out_4(3,3);
	Eigen::Matrix3d out_5(3,3);

	utils::rot_z(z_1, out_1);
	utils::rot_z(z_2, out_2);
	utils::rot_z(z_3, out_3);
	utils::rot_z(z_4, out_4);
	utils::rot_z(z_5, out_5);
	
	ASSERT_TRUE(out_1.isApprox(rot_1, TOL));
  ASSERT_TRUE(out_2.isApprox(rot_2, TOL));
	ASSERT_TRUE(out_3.isApprox(rot_3, TOL));
	ASSERT_TRUE(out_4.isApprox(rot_4, TOL));
	ASSERT_TRUE(out_5.isApprox(rot_5, TOL));
}

int 
main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
