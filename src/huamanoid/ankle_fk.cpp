#include "humanoid/ankle_fk.h"
#include <Eigen/src/Core/Matrix.h>

static Eigen::Vector2d f(double tx, double ty, double d, double L1, double h1, double h2, double tML, double tMR)
{
    Eigen::Vector2d err;
    // clang-format off
    double res1 = L1 * L1 + d * d - d* d*cos(tx) - L1* L1*cos(tML)*cos(ty) - L1* L1*sin(tML)*sin(ty) + L1*h1*sin(tML) - L1*h1*sin(ty) - d*h1*cos(ty)*sin(tx) + L1*d*cos(tML)*sin(tx)*sin(ty) - L1*d*cos(ty)*sin(tML)*sin(tx);
    double res2 = L1 * L1 + d * d - d* d*cos(tx) - L1* L1*cos(tMR)*cos(ty) - L1*L1*sin(tMR)*sin(ty) + L1*h2*sin(tMR) - L1*h2*sin(ty) + d*h2*cos(ty)*sin(tx) - L1*d*cos(tMR)*sin(tx)*sin(ty) + L1*d*cos(ty)*sin(tMR)*sin(tx);
    // clang-format on
    err << res1, res2;
    return err;
}

static Eigen::Matrix2d df(double tx, double ty, double d, double L1, double h1, double h2, double tML, double tMR)
{
    Eigen::Matrix2d df_mat;
    // clang-format off
    double df1_dx1 = d*d*sin(tx) - d*h1*cos(ty)*cos(tx) + L1*d*cos(tML)*cos(tx)*sin(ty) - L1*d*cos(ty)*sin(tML)*cos(tx);
    double df2_dx1 = d*d*sin(tx) + d*h2*cos(ty)*cos(tx) - L1*d*cos(tMR)*cos(tx)*sin(ty) + L1*d*cos(ty)*sin(tMR)*cos(tx);

    double df1_dx2 = L1*L1*cos(tML)*sin(ty) - L1*L1*sin(tML)*cos(ty) - L1*h1*cos(ty) + d*h2*sin(ty)*sin(tx) + L1*d*cos(tML)*sin(tx)*cos(ty) + L1*d*sin(ty)*sin(tML)*sin(tx);
    double df2_dx2 = L1*L1*cos(tMR)*sin(ty) - L1*L1*sin(tMR)*cos(ty) - L1*h2*cos(ty) - d*h2*sin(ty)*sin(tx) - L1*d*cos(tMR)*sin(tx)*cos(ty) - L1*d*sin(ty)*sin(tMR)*sin(tx);
    // clang-format on
    df_mat << df1_dx1, df1_dx2, df2_dx1, df2_dx2;
    return df_mat;
}

Eigen::Vector2d ankle_fk(double d, double L1, double h1, double h2, double mL, double mR)
{
    // simple newton's method
    int             count = 0;
    Eigen::Vector2d txy   = Eigen::Vector2d::Zero();
    Eigen::Vector2d err_v = f(txy(0), txy(1), d, h1, h2, L1, mL, mR);
    double          err   = err_v.transpose() * err_v;

    while (count < 20 && err > 1e-6)
    {
        Eigen::Matrix2d df_mat = df(txy(0), txy(1), d, h1, h2, L1, mL, mR);
        Eigen::Matrix2d J      = df_mat.inverse();
        Eigen::Vector2d f_    = f(txy(1), txy(2), d, h1, h2, L1, mL, mR);
        Eigen::Vector2d dxy    = -J * f_;
        txy                    = txy + dxy;
    }
    return txy;
}
