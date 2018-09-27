#include <pkg_utils/track.h>
#include <pkg_utils/eigen_helpers.h>

Eigen::Matrix<double, NP, NP>
R2(double phi)
{
    Eigen::Matrix<double, NP, NP> R;
    R << cos(phi), -sin(phi),
         sin(phi), cos(phi);
    return R;
}

Eigen::Matrix<double, 1, NP>
S1(Eigen::Matrix<double, NP, 1> p)
{
    Eigen::Matrix<double, 1, NP> c;
    c << p[1], -p[0];
    return c;
}

Eigen::Matrix<double, NP, NP>
S2(double omega)
{
    Eigen::Matrix<double, NP, NP> S;
    S << 0.0f, -1.0f,
         1.0f, 0.0f;
    return omega * S;
}

Eigen::Matrix<double, NNU, NNU>
Tx(double phi)
{
    Eigen::Matrix<double, NNU, NNU> T;
    T << R2(phi), Eigen::Matrix<double, NP, 1>::Zero(),
         Eigen::Matrix<double, 1, NP>::Zero(), 1.0f;
    return T;
}

Eigen::Matrix<double, NNU, NNU>
Yx(double omega)
{
    Eigen::Matrix<double, NNU, NNU> Y;
    Y << S2(omega), Eigen::Matrix<double, NP, 1>::Zero(),
         Eigen::Matrix<double, 1, NP>::Zero(), 0.0f;
    return Y;
}

Track::Track()
: Track(Eigen::MatrixXd::Zero(NP, 0), 0.0f, false)
{}

Track::Track(const Eigen::MatrixXd &Y, const double width, const bool closed)
: centerline(Y, getChordBreaks(Y, closed), Spline::CUBIC, closed)
, width(width)
{}

double
Track::evaluatePhi(double t)
{
    Eigen::Matrix<double, NP, 1> dc = this->centerline.evaluate(t, 1);
    return std::atan2(dc[1], dc[0]);
}

Nu
Track::evaluateNu(double t)
{
    Nu nu;
    nu << this->centerline.evaluate(t, 0),
          this->evaluatePhi(t);
    return nu;
}

Nu
Track::evaluateNuDerivative(double t)
{
    Nu dnu;
    Eigen::Matrix<double, NP, 1> dc = this->centerline.evaluate(t, 1);
    Eigen::Matrix<double, NP, 1> ddc = this->centerline.evaluate(t, 2);

    dnu << dc,
           S1(ddc) * dc / dc.dot(dc);
    return dnu;
}
