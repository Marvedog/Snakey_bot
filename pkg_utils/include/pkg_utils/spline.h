#ifndef _SPLINE_H
#define _SPLINE_H
#include <Eigen/Dense>

namespace utils
{

	class Ppoly {
	public:
			
			/* Piecewise polynomial of m segments */
			Ppoly(std::size_t dim, std::size_t deg, std::size_t m);
			
			Ppoly(std::size_t dim, std::size_t deg, Eigen::VectorXd breaks);
			Eigen::VectorXd pevaluate(double t, std::size_t derivative);
			const Eigen::MatrixXd &coefdata() const;
			const Eigen::VectorXd &breakdata() const;
			double end();

			std::size_t dim;
			std::size_t m;
			std::size_t deg;

			Eigen::VectorXd breaks;
			Eigen::MatrixXd coefs;
	private:
			std::size_t findSegment(double t);
	};

	class Spline : public Ppoly {
	public:
			
			/* Spline constructor 
			 * Construct a spline of pre-specified degree 
			 * Inputs:
			 *  			Y : Interpolation points. Number of rows decides number of dimensions
			 * 				breaks : progress along spline where points are patched together
			 * 				deg : polynomial degree for the patched splines
			 * 				closed : boolean deciding whether the spline should be closed or not. 
			*/
			Spline(const Eigen::MatrixXd &Y, const Eigen::VectorXd &t, const std::size_t deg, const bool closed);
			Spline();


			bool closed;

			void setChordLengthBreaks(Eigen::VectorXd l);
			double project(Eigen::VectorXd p, double t);
			double wrap(double t);
			Eigen::VectorXd evaluate(double t, std::size_t derivative);
			enum { ERROR = 0, LINEAR, QUADRATIC, CUBIC };
	private:
			
			/* Returns newton direction */
			double newton(Eigen::VectorXd p, double t);
			
			double evaluateDistanceSquared(Eigen::VectorXd p, double t);


			/* Function cubicA
			 * Generates the lhs of a system of equations of either a natural spline or a closed 
			 * spline dependent on the closed boolean
			 * Inputs:
			 * 				l : segment lengths
			 *				closed : true => closed spline, false => natural spline
			 * Outputs: 
			 * 				A : Triagonal matrix setting opp the lhs og the system of equations for a cubic spline
			*/
			static Eigen::MatrixXd cubicA(Eigen::VectorXd l, bool closed);
			
			/* Function cubicB
			 * Generates the rhs of a system of equations a natural spline or a closed spline 
			 * dependent on the closed boolean
			 * Inputs:
			 * 				Y : deg X m reference points
			 * 				l : segment lengths
			 *				closed : true => closed spline, false => natural spline
			 * Outputs: 
			 * 				B : Vector setting opp the rhs og the system of equations for a cubic spline
			*/
			static Eigen::MatrixXd cubicB(Eigen::MatrixXd Y, Eigen::VectorXd l, bool closed);
	};

	class Spline2d: public Spline {
	public:
			Spline2d(const Eigen::MatrixXd &Y, const Eigen::VectorXd &t, const std::size_t deg, const bool closed);
			Spline2d();
			double evaluateCurvature(double t);
	};

} /* End namespace utils */

#endif
