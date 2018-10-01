#include "pkg_utils/spline.h"
#include "pkg_utils/eigen_helpers.h"
#include "pkg_utils/helper_functions.h"

namespace utils
{

	/* Orthogonal projection of a point onto a curve as described in
	*  a second order algorithm for orthogonal projection onto curves
	*  and surfaces by [Shi-min Hu and Johannes Wallner] */

	Eigen::VectorXd
	Spline::evaluate(double t, std::size_t derivative)
	{
			t = this->wrap(t);
			return this->pevaluate(t, derivative);
	}

	double
	Spline::evaluateDistanceSquared(Eigen::VectorXd p, double t)
	{
			Eigen::VectorXd tmp = this->evaluate(t, 0) - p;
			return tmp.dot(tmp);
	}

	double
	Spline::newton(Eigen::VectorXd p, double t)
	{
			Eigen::VectorXd c = this->evaluate(t, 0);
			Eigen::VectorXd dc = this->evaluate(t, 1);
			Eigen::VectorXd ddc = this->evaluate(t, 2);

			return dc.dot(p - c) / (ddc.dot(c) + dc.dot(dc) - p.dot(ddc));
	}

	double
	Spline::project(Eigen::VectorXd p, double t)
	{
			double dt;
			const double tol = 1.0e-6;
			const double alpha = 0.9f;
			do {
					double d = this->evaluateDistanceSquared(p, t);
					dt = newton(p, t);

					/* Short circuit if projecting onto ends of open splines */
					if (!this->closed && t + dt < 0.0f) {
							dt = -t;
					} else if (!this->closed && t + dt > this->end()) {
							dt = this->end() - t;
					}            
					
					/* Perform line search */
					while (this->evaluateDistanceSquared(p, t + dt) > d) {
							dt *= alpha;
					}
					t += dt;
			} while (fabs(dt) > tol);
			return t;
	}

	double
	Spline::wrap(double t)
	{
			if (this->closed) {
					/* More efficient floating point modulo according to some source */
					for ( ; t < 0; t += this->end());
					for ( ; t > this->end(); t -= this->end());
			} else {
					t = saturate(t, 0, this->end());
			}
			return t;
	}

	Eigen::VectorXd
	Ppoly::pevaluate(double t, std::size_t derivative)
	{
			std::size_t j = this->findSegment(t);
			Eigen::VectorXd u = this->coefs.col(j);

			Eigen::VectorXd v = Eigen::VectorXd::Zero(this->dim);
			for (std::size_t i = 0; i + derivative < this->deg + 1; ++i) {
					v = v*(t - this->breaks[j]) + partialFactorial(this->deg - i, derivative)*u.segment(this->dim*i, this->dim);
			}
			return v;
	}

	Ppoly::Ppoly(std::size_t dim, std::size_t deg, std::size_t m)
	: coefs(dim*(deg + 1), m)
	, breaks(m + 1)
	, dim(dim)
	, deg(deg)
	, m(m)
	{}

	Ppoly::Ppoly(std::size_t dim, std::size_t deg, Eigen::VectorXd breaks)
	: m(breaks.size() > 1 ? breaks.size() - 1: 0)
	, breaks(breaks)
	, coefs(dim*(deg + 1), m)
	, dim(dim)
	, deg(deg)
	{}


	double
	Ppoly::end()
	{
			return this->breaks[this->m];
	}

	const Eigen::MatrixXd &
	Ppoly::coefdata() const
	{
			return this->coefs;
	}

	const Eigen::VectorXd &
	Ppoly::breakdata() const
	{
			return this->breaks;
	}

	std::size_t
	Ppoly::findSegment(double t)
	{
			std::size_t i;
			for (i = 0; this->breaks[i + 1] < t; i = (i + 1) % this->m);
			return i;
	}

	/* Sets chordal length break points and return vector of segment lengths */
	void
	Spline::setChordLengthBreaks(Eigen::VectorXd l)
	{
			this->breaks[0] = 0;
			for (std::size_t i = 0; i < this->m; ++i) {
					this->breaks[i + 1] = this->breaks[i] + l[i];
			}
	}

	Spline::Spline()
	: Spline(Eigen::MatrixXd::Zero(0, 0), Eigen::VectorXd::Zero(0), Spline::ERROR, false)
	{}


	Spline::Spline(const Eigen::MatrixXd &Y, const Eigen::VectorXd &breaks, const std::size_t deg, const bool closed)
	: Ppoly(Y.rows(), deg, breaks)
	, closed(closed)
	{
			switch (deg) {
			case CUBIC:
					if (this->m >= CUBIC) {
							Eigen::VectorXd l = breaks.tail(breaks.size() - 1) - breaks.head(breaks.size() - 1);
							Eigen::MatrixXd T = cubicA(l, closed);
							Eigen::MatrixXd F = cubicB(Y, l, closed);
							Eigen::MatrixXd DDY = T.transpose().colPivHouseholderQr().solve(F.transpose()).transpose();

							Eigen::MatrixXd YP = cycleMatrix(Y, 0, -1);
							Eigen::MatrixXd DDYP = cycleMatrix(DDY, 0, -1);

							Eigen::MatrixXd DDYDIFF = DDYP - DDY;
							Eigen::MatrixXd DDYSUM = DDYP + 2.0f*DDY;

							Eigen::MatrixXd ltmp = Eigen::MatrixXd::Constant(Y.rows(), 1, 1.0f) * l.transpose();
							Eigen::MatrixXd YDIFF = YP - Y;

							Eigen::MatrixXd A = DDYDIFF.leftCols(this->m).array() / (6.0f*ltmp.array());
							Eigen::MatrixXd B = 1/2.0f*DDY.leftCols(this->m);
							Eigen::MatrixXd C = YDIFF.leftCols(this->m).array() / ltmp.array() - ltmp.array() / 6.0f*DDYSUM.leftCols(this->m).array();
							Eigen::MatrixXd D = Y.leftCols(this->m);

							this->coefs << A, B, C, D;
							break;
					} else {
							this->deg = LINEAR;
							this->coefs.resize(this->dim*(LINEAR + 1), this->m);
					}
			case LINEAR:
					if (this->m >= LINEAR) {
							Eigen::VectorXd l = breaks.tail(breaks.size() - 1) - breaks.head(breaks.size() - 1);
							Eigen::MatrixXd YP = cycleMatrix(Y, 0, -1);
							Eigen::MatrixXd YDIFF = YP - Y;

							Eigen::MatrixXd ltmp = Eigen::MatrixXd::Constant(Y.rows(), 1, 1.0f) * l.transpose();

							Eigen::MatrixXd A = YDIFF.leftCols(this->m).array() / ltmp.array();
							Eigen::MatrixXd B = Y.leftCols(this->m);

							this->coefs << A, B;
							break;
					}
			default:
					this->deg = ERROR;
					return;
			}

	}


	Eigen::MatrixXd
	Spline::cubicA(Eigen::VectorXd l, bool closed)
	{
			size_t m = l.rows();

			Eigen::VectorXd ln = cycleVector(l, 1);
			Eigen::VectorXd tmp = ln + l;

			Eigen::MatrixXd A = 1/3.0f * tmp.asDiagonal().toDenseMatrix();
			A.diagonal(1) = 1/6.0f * l.head(m - 1);
			A.diagonal(-1) = 1/6.0f * l.head(m - 1);

			if (closed) {
					A(m - 1, 0) = 1/6.0f * l[m - 1];
					A(0, m - 1) = 1/6.0f * l[m - 1];
			} else {
					A(0, 0) = 1/3.0f * l[0];
					A(m - 1, m - 1) = 1/3.0f * l[m - 1];
			}
			return A;
	}


	Eigen::MatrixXd
	Spline::cubicB(Eigen::MatrixXd Y, Eigen::VectorXd l, bool closed)
	{
			size_t m = l.rows();

			Eigen::VectorXd ln = cycleVector(l, 1);
			Eigen::MatrixXd YN = cycleMatrix(Y, 0, 1);
			Eigen::MatrixXd YP = cycleMatrix(Y, 0, -1);

			Eigen::MatrixXd C = YP - Y;
			Eigen::MatrixXd D = Y - YN;

			Eigen::MatrixXd ltmp = Eigen::MatrixXd::Constant(Y.rows(), 1, 1.0f) * l.transpose();
			Eigen::MatrixXd lntmp = Eigen::MatrixXd::Constant(Y.rows(), 1, 1.0f) * ln.transpose();

			Eigen::MatrixXd B = C.leftCols(m).array() / ltmp.array() - D.leftCols(m).array() / lntmp.array();

			if (!closed) {
					/*B.col(0) = - (Y.col(1) - Y.col(0)) / l[0];
					B.col(m - 1) = (Y.col(m) - Y.col(m - 1)) / l[m - 1];*/
					B.col(0).setZero();
					B.col(m - 1).setZero();
			}
			return B;
	}

	Spline2d::Spline2d()
	: Spline()
	{}

	Spline2d::Spline2d(const Eigen::MatrixXd &Y, const Eigen::VectorXd &breaks, const std::size_t deg, const bool closed)
	: Spline(Y, breaks, Y.rows() == 2 ? deg: Spline::ERROR, closed)
	{}

	double
	Spline2d::evaluateCurvature(double t)
	{
			if (this->deg == Spline::ERROR) {
					return std::nan("0");
			}
			Eigen::VectorXd dc = this->evaluate(t, 1);
			Eigen::VectorXd ddc = this->evaluate(t, 2);
			std::size_t dim = dc.size();
			/* Cross product is not defined for dimensions lower \
			 * than 3, this workaround is ugly */
			Eigen::MatrixXd tmp(2, 2);
			tmp << dc, ddc;
			double n = dc.norm();

			return tmp.determinant() / n / n / n;
	}

} /* End namespace utils */
