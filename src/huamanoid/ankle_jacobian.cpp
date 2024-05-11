#include "humanoid/ankle_jacobian.h"
#include "humanoid/ankle_ik.h"

Eigen::Matrix2d ankle_jacobian(double d, double L1, double h1, double h2, double tx, double ty)
{
    // TODO : improve this with symbolic result
    Eigen::Matrix2d J;

    double dtx = tx > 0 ? -1e-5 : 1e-5;
    double dty = ty > 0 ? -1e-5 : 1e-5;

    auto   m   = ankle_ik(d, L1, h1, h2, tx, ty);
    auto   mx  = ankle_ik(d, L1, h1, h2, tx + dtx, ty);
    auto   my  = ankle_ik(d, L1, h1, h2, tx, ty + dty);

    J(0, 0) = (mx[0] - m[0]) / dtx;
    J(0, 1) = (my[0] - m[0]) / dty;
    J(1, 0) = (mx[1] - m[1]) / dtx;
    J(1, 1) = (my[1] - m[1]) / dty;

    return J;
}
