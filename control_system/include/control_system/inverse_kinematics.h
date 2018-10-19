#include <Eigen/Dense>
#include <pkg_utils/jacobian.h>
//#include <pkg_utils/modules.h>
#include <vector>

/// Class computing inverse kinematics of rear end-effector
/** 
	* TODO: GENERALIZE FOR ANY POINT TO BE BASE
	* @inputs have to be correct in sizes. See doc. => Make reference
	* @reference transform vectors
	* @head is link 0
	* @tail is link (links-1)
	* @homogeneous transformations from head to tail are store
	* @inverse call to each module provide tranformations the other way
	*/

class InverseKinematics
{
	public:
		InverseKinematics();
		InverseKinematics( int links
		   							 , int joints
										 , std::vector<double> a0_joints
										 , std::vector<double> a1_joints
										 , std::vector<double> a2_joints
										 , std::vector<double> q1_joints
										 , std::vector<double> q2_joints
										 , std::vector<double> alpha0_joints
										 , std::vector<double> l_links
										 , std::vector<int> snake_config); 

		Eigen::MatrixXd computeGeometricJacobian( const std::vector<double> &q1
																						, const std::vector<double> &q2);

	private:
	
		/// Computes transformation from base to tail given stored transformations	
		Eigen::Matrix4d baseToTail();
		Eigen::Matrix4d tailToBase();

		/// Compute transformation
		Eigen::Matrix4d transformBetween(int a, int b);		

		/// Geometric jacobian for front end-effector
		Eigen::MatrixXd geometric_jacobian_f;
		
		/// Geometric jacobian for rear end-effector
		Eigen::MatrixXd geometric_jacobian_r;

		/// List of snake configuration [1 1 0 1 0 1] => [link link joint link joint link ]
		std::vector<int> snake_config;	
		//std::vector<JointModule> joint_modules;
		//std::vector<LinkModule> link_modules;

		/// Vector of transforms
		std::vector<Eigen::Matrix4d> base_to_joints;
		std::vector<Eigen::Matrix4d> base_to_joints_zero;
};
