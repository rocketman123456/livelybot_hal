#include "humanoid/ankle_jacobian.h"
#include "humanoid/ankle_ik.h"

Eigen::Matrix2d ankle_jacobian(double d, double L1, double h1, double h2, double tx, double ty)
{
    Eigen::Matrix2d J;

    double dt = 1e-5;
    auto   m = ankle_ik(d, L1, h1, h2, tx, ty);
    auto   mx = ankle_ik(d, L1, h1, h2, tx+dt, ty);
    auto   my = ankle_ik(d, L1, h1, h2, tx, ty+dt);

    J(0, 0) = (mx[0] - m[0]) / dt;
    J(0, 1) = (my[0] - m[0]) / dt;
    J(1, 0) = (mx[1] - m[1]) / dt;
    J(1, 1) = (my[1] - m[1]) / dt;

    return J;
}
