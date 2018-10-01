#ifndef _TRACK_H
#define _TRACK_H
#include <pkg_utils/spline.h>

namespace utils
{

	/* NP is */
	const std::size_t NP = 2;
	
	/* NP is */
	const std::size_t NNU = 3;
	typedef Eigen::Matrix<double, NNU, 1> Nu; /* Column vector */

	/* NP is */
	Eigen::Matrix<double, NP, NP> R2(double phi);
	
	/* NP is */
	Eigen::Matrix<double, 1, NP> S1(Eigen::Matrix<double, NP, 1> p);
	
	/* NP is */
	Eigen::Matrix<double, NP, NP> S2(double omega);
	
	/* NP is */
	Eigen::Matrix<double, NNU, NNU> Tx(double phi);
	
	/* NP is */
	Eigen::Matrix<double, NNU, NNU> Yx(double omega);

	class Track {
	public:
			Track();
			Track(const Eigen::MatrixXd &Y, const double width, const bool open);
			double evaluatePhi(double t);
			Nu evaluateNu(double t);
			Nu evaluateNuDerivative(double t);

			Spline centerline;
			double width;
	};

} /* End namespace utils */
#endif
