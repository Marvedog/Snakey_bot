#include "pkg_utils/eigen_helpers.h"
#include <gtest/gtest.h>


TEST(EigenHelpersTest, CycleBothDims) {
    Eigen::MatrixXd U(3, 3);
    U << 1, 2, 3, 4, 5, 6, 7, 8, 9;

    Eigen::MatrixXd W(3, 3);
    W << 9, 7, 8, 3, 1, 2, 6, 4, 5;

    ASSERT_TRUE(cycleMatrix(U, 1, 1) == cycleMatrix(U, -2, -2));
    ASSERT_TRUE(cycleMatrix(U, 1, 1) == W);
}

TEST(EigenHelpersTest, CycleRows) {
    Eigen::MatrixXd U(3, 3);
    U << 1, 2, 3, 4, 5, 6, 7, 8, 9;

    Eigen::MatrixXd W(3, 3);
    W << 7, 8, 9, 1, 2, 3, 4, 5, 6;

    ASSERT_TRUE(cycleMatrix(U, 1, 0) == cycleMatrix(U, -2, 0));
    ASSERT_TRUE(cycleMatrix(U, 1, 0) == W);
}

TEST(EigenHelpersTest, CycleCols) {
    Eigen::MatrixXd U(3, 3);
    U << 1, 2, 3, 4, 5, 6, 7, 8, 9;

    Eigen::MatrixXd W(3, 3);
    W << 3, 1, 2, 6, 4, 5, 9, 7, 8;

    ASSERT_TRUE(cycleMatrix(U, 0, 1) == cycleMatrix(U, 0, -2));
    ASSERT_TRUE(cycleMatrix(U, 0, 1) == W);
}

TEST(EigenHelpersTest, CycleVector) {
    Eigen::VectorXd u(9);
    u << 1, 2, 3, 4, 5, 6, 7, 8, 9;

    Eigen::VectorXd w(9);
    w << 9, 1, 2, 3, 4, 5, 6, 7, 8;

    ASSERT_TRUE(cycleVector(u, 1) == cycleVector(u, -8));
    ASSERT_TRUE(cycleVector(u, 1) == w);
}

TEST(EigenHelpersTest, Converters) {
    Eigen::MatrixXd m0(2, 4);
    m0 << 1, 3, 5, 7,
          2, 4, 6, 8;

    std_msgs::Float64MultiArray ma = matrixToMultiArray(m0);
    Eigen::MatrixXd m1 = multiArrayToMatrix(ma);
    std_msgs::Float64MultiArray ma1 = matrixToMultiArray(m1);

    ASSERT_TRUE(m1 == m0);

    for (int r=0; r<2; r++) {
        or (int c=0; c<4; c++) {
            ASSERT_TRUE(m0(r,c) == m1(r,c));
        }
    }
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
