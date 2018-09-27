#include "pkg_utils/spline.h"
#include "pkg_utils/eigen_helpers.h"
#include <gtest/gtest.h>


//TEST(SplineTest, B) {
//    Eigen::MatrixXd Y(2, 4);
//    Y << 1.2f, 0.1f, -1.3f, -0.2f,
//         -0.2f, 0.9f, 0.1f, -1.4f;
//
//    Eigen::VectorXd l = getChordLengths(Y);
//    std::cout << l << std::endl;
//
//    Eigen::MatrixXd B(2, 4);
//    B << -1.4664f, -0.1611f, 1.4596f, 0.1679f,
//         0.0563f, -1.2032f, -0.3103f, 1.4572f;
//
//    std::cout << Cubicspline::B(Y, l);
//    Eigen::MatrixXd TMP = Cubicspline::B(Y, l) - B;
//    ASSERT_LT(TMP.norm(), 0.01f);
//}
//
//TEST(SplineTest, A) {
//    Eigen::MatrixXd Y(2, 4);
//    Y << 1, 0, -1, 0,
//         0, 1, 0, -1;
//    Eigen::VectorXd l = getChordLengths(Y);
//    Eigen::MatrixXd A = Eigen::VectorXd::Constant(4, 0.9428f).asDiagonal().toDenseMatrix();
//    A.diagonal(1) = Eigen::VectorXd::Constant(3, 0.2357f);
//    A.diagonal(-1) = Eigen::VectorXd::Constant(3, 0.2357f);
//
//    A(3, 0) = 0.2357f;
//    A(0, 3) = 0.2357f;
//
//    Eigen::MatrixXd TMP = Cubicspline::A(l) - A;
//    ASSERT_LT(TMP.norm(), 0.01f);
//}
//
//TEST(SplineTest, Evaluation) {
//    Eigen::MatrixXd Y(2, 4);
//    Y << 1, 0, -1, 0,
//         0, 1, 0, -1;
//    Cubicspline cspline(Y, true);
//
//    Eigen::VectorXd v(2);
//    v << 0.4268f, 0.8839f;
//    std::cout << cspline.evaluate(1.0f, 0);
//    Eigen::VectorXd tmp = v - cspline.evaluate(1.0f, 0);
//
//    ASSERT_LT(tmp.norm(), 0.01f);
//}
//
//TEST(SplineTest, DDY) {
//    Eigen::MatrixXd Y(2, 4);
//    Y << 1, 0, -1, 0,
//         0, 1, 0, -1;
//
//    Eigen::MatrixXd DDY(2, 4);
//    DDY << -1.5f, 0.0f, 1.5f, 0.0f,
//           0.0f, -1.5f, 0.0f, 1.5f;
//
//    Eigen::VectorXd l = getChordLengths(Y);
//    Eigen::MatrixXd T = Cubicspline::A(l);
//    Eigen::MatrixXd F = Cubicspline::B(Y, l);
//    Eigen::MatrixXd TMP = DDY - T.transpose().fullPivLu().solve(F.transpose()).transpose();
//    ASSERT_LT(TMP.norm(), 0.01f);
//}
//
//TEST(SplineTest, CubicToMsg) {
//    Eigen::MatrixXd Y(2, 4);
//    Y << 1, 0, -1, 0,
//         0, 1, 0, -1;
//
//    Cubicspline spline = Cubicspline(Y, true);
//    pkg_msgs::Spline msg = spline.toSplineMsg();
//
//    Cubicspline result = Cubicspline(msg);
//    ASSERT_EQ(spline.end(), result.end());
//    ASSERT_EQ(spline.evaluate(1.0, 0), result.evaluate(1.0, 0));
//}

TEST(SplineTest, OneDimSplineCtor) {
    Eigen::MatrixXd Y(1, 2);
    Y << 0, 1;

    Eigen::VectorXd V(2);
    V << 0, 1;

    Spline spline = Spline(Y, V, Spline::CUBIC, false);
    ASSERT_FALSE(std::isnan(spline.evaluate(1,0).norm()));
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
