#include "humanoid/ankle_fk.h"
#include <Eigen/src/Core/Matrix.h>

static Eigen::Vector2d f(double tx, double ty, double d, double L1, double h1, double h2, double tML, double tMR)
{
    Eigen::Vector2d err;
    double cx = cos(tx);
    double sx = sin(tx);
    double cy = cos(ty);
    double sy = sin(ty);
    double cml = cos(tML);
    double sml = sin(tML);
    double cmr = cos(tMR);
    double smr = sin(tMR);
    // clang-format off
    double res1 = L1*L1 + d*d - d*d*cx - L1*L1*cml*cy - L1*L1*sml*sy + L1*h1*sml - L1*h1*sy - d*h1*cy*sx + L1*d*cml*sx*sy - L1*d*cy*sml*sx;
    double res2 = L1*L1 + d*d - d*d*cx - L1*L1*cmr*cy - L1*L1*smr*sy + L1*h2*smr - L1*h2*sy + d*h2*cy*sx - L1*d*cmr*sx*sy + L1*d*cy*smr*sx;
    // clang-format on
    err << res1, res2;
    return err;
}

static Eigen::Matrix2d df(double tx, double ty, double d, double L1, double h1, double h2, double tML, double tMR)
{
    Eigen::Matrix2d df_mat;
    double cx = cos(tx);
    double sx = sin(tx);
    double cy = cos(ty);
    double sy = sin(ty);
    double cml = cos(tML);
    double sml = sin(tML);
    double cmr = cos(tMR);
    double smr = sin(tMR);
    // clang-format off
    double df1_dx1 = d*d*sx - d*h1*cy*cx + L1*d*cml*cx*sy - L1*d*cy*sml*cx;
    double df2_dx1 = d*d*sx + d*h2*cy*cx - L1*d*cmr*cx*sy + L1*d*cy*smr*cx;

    double df1_dx2 = L1*L1*cml*sy - L1*L1*sml*cy - L1*h1*cy + d*h2*sy*sx + L1*d*cml*sx*cy + L1*d*sy*sml*sx;
    double df2_dx2 = L1*L1*cmr*sy - L1*L1*smr*cy - L1*h2*cy - d*h2*sy*sx - L1*d*cmr*sx*cy - L1*d*sy*smr*sx;
    // clang-format on
    df_mat << df1_dx1, df1_dx2, df2_dx1, df2_dx2;
    return df_mat;
}

Eigen::Vector2d ankle_fk(double d, double L1, double h1, double h2, double mL, double mR)
{
    // TODO : improve this with symbolic result
    // simple newton's method
    int             count = 0;
    Eigen::Vector2d txy   = Eigen::Vector2d::Zero();
    Eigen::Vector2d err_v = f(txy(0), txy(1), d, h1, h2, L1, mL, mR);
    double          err   = err_v.transpose() * err_v;

    while (count < 10 && err > 1e-3)
    {
        Eigen::Matrix2d df_mat = df(txy(0), txy(1), d, h1, h2, L1, mL, mR);
        Eigen::Matrix2d J      = df_mat.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::Vector2d f_    = f(txy(0), txy(1), d, h1, h2, L1, mL, mR);
        Eigen::Vector2d dxy    = -J * f_;
        txy                    = txy + dxy;
        count++;
    }
    return txy;
}
