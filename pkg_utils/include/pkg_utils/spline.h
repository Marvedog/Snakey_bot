#ifndef _SPLINE_H
#define _SPLINE_H
#include <Eigen/Dense>

class Ppoly {
public:
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
    Spline(const Eigen::MatrixXd &Y, const Eigen::VectorXd &t, const std::size_t deg, const bool closed);
    Spline();


    bool closed;

    void setChordLengthBreaks(Eigen::VectorXd l);
    double project(Eigen::VectorXd p, double t);
    double wrap(double t);
    Eigen::VectorXd evaluate(double t, std::size_t derivative);
    enum { ERROR = 0, LINEAR, QUADRATIC, CUBIC };
private:
    double newton(Eigen::VectorXd p, double t);
    double evaluateDistanceSquared(Eigen::VectorXd p, double t);


    static Eigen::MatrixXd cubicA(Eigen::VectorXd l, bool closed);
    static Eigen::MatrixXd cubicB(Eigen::MatrixXd Y, Eigen::VectorXd l, bool closed);
};

class Spline2d: public Spline {
public:
    Spline2d(const Eigen::MatrixXd &Y, const Eigen::VectorXd &t, const std::size_t deg, const bool closed);
    Spline2d();
    double evaluateCurvature(double t);
};

#endif
