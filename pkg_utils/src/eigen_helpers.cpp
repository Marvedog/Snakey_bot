#include <fstream>
#include <Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

namespace utils
{

	Eigen::MatrixXd
	readWaypoints(std::string path)
	{
			size_t n = 0;
			Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(2, n);
			std::ifstream fin(path);

			double x, y;
			while (fin >> x, fin >> y) {
					Y.conservativeResize(Eigen::NoChange, n + 1);
					Y.col(n++) << x, y;
			}
			fin.close();
			return Y;
	}

	Eigen::MatrixXd
	cycleMatrix(const Eigen::MatrixXd &X, int drow, int dcol)
	{
			std::size_t m = X.rows();
			std::size_t n = X.cols();
			if (m*n == 0)
					return X;
			if (drow < 0)
					drow = m + drow;
			if (dcol < 0)
					dcol = n + dcol;

			Eigen::MatrixXd A = X.bottomRightCorner(drow, dcol);
			Eigen::MatrixXd B = X.bottomLeftCorner(drow, n - dcol);
			Eigen::MatrixXd C = X.topRightCorner(m - drow, dcol);
			Eigen::MatrixXd D = X.topLeftCorner(m - drow, n - dcol);

			/* Have to be fixed for the general case,
				 concatining zero dimensional matrices
				 seems to cause problems for the
				 ','-operator. */
			/* Temporary, (ugly?) fix is in place,
				 feel free to improve upon this. */

			Eigen::MatrixXd Y(X.rows(), X.cols());
			if (drow == 0 && dcol == 0) {
					Y = X;
			} else if (drow == 0) {
					Y << C, D;
			} else if (dcol == 0) {
					Y << B, D;
			} else {
					Y << A, B, C, D;
			}

			return Y;
	}

	Eigen::VectorXd
	cycleVector(const Eigen::VectorXd &x, int drow)
	{
			std::size_t m = x.rows();
			Eigen::VectorXd y(x.rows());

			if (drow == 0) {
					y = x;
			} else {
					if (drow < 0)
							drow = m + drow;

					Eigen::MatrixXd a = x.tail(drow);
					Eigen::MatrixXd b = x.head(m - drow);
					y << a, b;
			}
			return y;
	}

	Eigen::VectorXd
	getChordLengths(const Eigen::MatrixXd &Y)
	{
			std::size_t m = Y.cols();
			Eigen::MatrixXd YP = cycleMatrix(Y, 0, -1);
			Eigen::MatrixXd D = YP - Y;
			
			Eigen::MatrixXd Dsquared = D.array() * D.array();
			Eigen::MatrixXd tmp = Eigen::MatrixXd::Constant(1, Y.rows(), 1.0f) * Dsquared;
			Eigen::VectorXd l = tmp.array().sqrt().transpose();
			return l;
	}

	Eigen::VectorXd
	getChordBreaks(const Eigen::MatrixXd &Y, bool closed)
	{
			/* Number of segments */
			std::size_t m = closed || Y.cols() == 0 ? Y.cols(): Y.cols() - 1; 
			
			/* Pre-allocation
				 TODO: Make this RT friendly
			*/
			Eigen::VectorXd t(m + 1);
			t[0] = 0.0f;
			
			/* Compute chord breaks 
				 TODO: Make this RT friendly
				 TODO: Hva er formålet med modulus her??  
			*/
			for (std::size_t i = 0; i < m; ++i) {
					Eigen::VectorXd diff = Y.col((i + 1) % Y.cols()) - Y.col(i);
					t[i+1] = t[i] + diff.norm();
			}
			return t;
	}

	std_msgs::Float64MultiArray
	matrixToMultiArray(const Eigen::MatrixXd e)
	{
			std_msgs::Float64MultiArray m;
			if (m.layout.dim.size() != 2)
					m.layout.dim.resize(2);
			m.layout.dim[0].stride = e.rows() * e.cols();
			m.layout.dim[0].size = e.rows();
			m.layout.dim[1].stride = e.cols();
			m.layout.dim[1].size = e.cols();

			int ii = 0;
			for (int i = 0; i < e.cols(); ++i)
					for (int j = 0; j < e.rows(); ++j)
							m.data.push_back(e.coeff(j, i));

			return m;
	}

	Eigen::MatrixXd
	multiArrayToMatrix(std_msgs::Float64MultiArray msg) {
			uint dstride0 = msg.layout.dim[0].stride;
			uint dstride1 = msg.layout.dim[1].stride;
			uint rows = msg.layout.dim[0].size;
			uint cols = msg.layout.dim[1].size;

			std::vector<double> data = msg.data;
			Eigen::Map<Eigen::MatrixXd> mat(data.data(), msg.layout.dim[0].size, msg.layout.dim[1].size);

			return mat;
	}

} /* End namespace utils */
