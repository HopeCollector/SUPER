/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#include <utils/optimization/mvie.h>
#include "utils/optimization/sdlp.h"
#include "utils/optimization/lbfgs.h"
#include <cfloat>

namespace optimization_utils {
    using namespace math_utils;
    using namespace geometry_utils;

    void MVIE::chol3d(const Eigen::Matrix3d &A, Eigen::Matrix3d &L) {
        L(0, 0) = sqrt(A(0, 0));
        L(0, 1) = 0.0;
        L(0, 2) = 0.0;
        L(1, 0) = 0.5 * (A(0, 1) + A(1, 0)) / L(0, 0);
        L(1, 1) = sqrt(A(1, 1) - L(1, 0) * L(1, 0));
        L(1, 2) = 0.0;
        L(2, 0) = 0.5 * (A(0, 2) + A(2, 0)) / L(0, 0);
        L(2, 1) = (0.5 * (A(1, 2) + A(2, 1)) - L(2, 0) * L(1, 0)) / L(1, 1);
        L(2, 2) = sqrt(A(2, 2) - L(2, 0) * L(2, 0) - L(2, 1) * L(2, 1));
        return;
    }

    bool MVIE::smoothedL1(const double &mu, const double &x, double &f, double &df) {
        if (x < 0.0) {
            return false;
        } else if (x > mu) {
            f = x - 0.5 * mu;
            df = 1.0;
            return true;
        } else {
            const double xdmu = x / mu;
            const double sqrxdmu = xdmu * xdmu;
            const double mumxd2 = mu - 0.5 * x;
            f = mumxd2 * sqrxdmu * xdmu;
            df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
            return true;
        }
    }


    double MVIE::costMVIE(void *data, const Eigen::VectorXd &x, Eigen::VectorXd &grad) {
        const int64_t *pM = (int64_t *) data;
        const double *pSmoothEps = (double *) (pM + 1);
        const double *pPenaltyWt = pSmoothEps + 1;
        const double *pA = pPenaltyWt + 1;

        const int M = *pM;
        const double smoothEps = *pSmoothEps;
        const double penaltyWt = *pPenaltyWt;
        Eigen::Map<const Eigen::MatrixX3d> A(pA, M, 3);
        Eigen::Map<const Eigen::Vector3d> p(x.data());
        Eigen::Map<const Eigen::Vector3d> rtd(x.data() + 3);
        Eigen::Map<const Eigen::Vector3d> cde(x.data() + 6);
        Eigen::Map<Eigen::Vector3d> gdp(grad.data());
        Eigen::Map<Eigen::Vector3d> gdrtd(grad.data() + 3);
        Eigen::Map<Eigen::Vector3d> gdcde(grad.data() + 6);

        double cost = 0;
        gdp.setZero();
        gdrtd.setZero();
        gdcde.setZero();

        Eigen::Matrix3d L;
        L(0, 0) = rtd(0) * rtd(0) + DBL_EPSILON;
        L(0, 1) = 0.0;
        L(0, 2) = 0.0;
        L(1, 0) = cde(0);
        L(1, 1) = rtd(1) * rtd(1) + DBL_EPSILON;
        L(1, 2) = 0.0;
        L(2, 0) = cde(2);
        L(2, 1) = cde(1);
        L(2, 2) = rtd(2) * rtd(2) + DBL_EPSILON;

        const Eigen::MatrixX3d AL = A * L;
        const Eigen::VectorXd normAL = AL.rowwise().norm();
        const Eigen::Matrix3Xd adjNormAL = (AL.array().colwise() / normAL.array()).transpose();
        const Eigen::VectorXd consViola = (normAL + A * p).array() - 1.0;

        double c, dc;
        Eigen::Vector3d vec;
        for (int i = 0; i < M; ++i) {
            if (smoothedL1(smoothEps, consViola(i), c, dc)) {
                cost += c;
                vec = dc * A.row(i).transpose();
                gdp += vec;
                gdrtd += adjNormAL.col(i).cwiseProduct(vec);
                gdcde(0) += adjNormAL(0, i) * vec(1);
                gdcde(1) += adjNormAL(1, i) * vec(2);
                gdcde(2) += adjNormAL(0, i) * vec(2);
            }
        }
        cost *= penaltyWt;
        gdp *= penaltyWt;
        gdrtd *= penaltyWt;
        gdcde *= penaltyWt;

        cost -= log(L(0, 0)) + log(L(1, 1)) + log(L(2, 2));
        gdrtd(0) -= 1.0 / L(0, 0);
        gdrtd(1) -= 1.0 / L(1, 1);
        gdrtd(2) -= 1.0 / L(2, 2);

        gdrtd(0) *= 2.0 * rtd(0);
        gdrtd(1) *= 2.0 * rtd(1);
        gdrtd(2) *= 2.0 * rtd(2);

        return cost;
    }

    // 参见笔记 notes/CIRI.md#Maximum Volume Inscribed Ellipsoid (Algorithm1-line8)
    bool MVIE::maxVolInsEllipsoid(const Eigen::MatrixX4d &hPoly, Ellipsoid &ellipsoid) {
        // 1. 取出当前椭球的旋转矩阵R、半轴长度r、中心p
        Mat3f R = ellipsoid.R();
        Vec3f r = ellipsoid.r();
        Vec3f p = ellipsoid.d();
        // Find the deepest interior point
        // 2. 计算多面体的面数 M
        const int M = hPoly.rows();

        Eigen::MatrixX4d Alp(M, 4);
        Eigen::VectorXd blp(M);
        Eigen::Vector4d clp, xlp;
        // 3.归一化每个超平面的法向量 A，并同步缩放偏移量 b
        const Eigen::ArrayXd hNorm = hPoly.leftCols<3>().rowwise().norm();
        Alp.leftCols<3>() = hPoly.leftCols<3>().array().colwise() / hNorm;
        Alp.rightCols<1>().setConstant(1.0);
        blp = -hPoly.rightCols<1>().array() / hNorm;

        // 4. 线性规划目标向量，目标是最大化深度
        clp.setZero();
        clp(3) = -1.0;

        // 5. 求解多面体内部最深的点（最大深度内点），作为椭球初始中心
        //    sdlp 是浙大 FAST 团队的线性规划求解器
        //    专门用于解决 min (c^T x), s.t. Ax <= b 的问题
        /*
            🧔：const double maxdepth = -sdlp::linprog<4>(clp, Alp, blp, xlp);
                上面这句话的 clp 是 [0, 0, 0, -1]，它表示优化目标就是 -x3，为什么说这个东西是深度？或者说为什么 x3 表示了 x 到各个平面的距离？
            
            🤖：这是一个关于多面体内部“最深点”线性规划的经典问题。
                1. H-Polytope 的定义
                hPoly 的每一行是 [a₁, a₂, a₃, b]，表示超平面 a·x + b ≤ 0。
                Alp 归一化后，变成 â·x + 1·x₃ ≤ b̂。

                2. 变量含义
                xlp = [x₁, x₂, x₃, d]，其中 d = x₃（最后一维），前面三维是空间点坐标。

                3. 线性规划的约束
                归一化后，约束变成： â·x + d ≤ b̂
                ⇔ â·x ≤ b̂ - d
                ⇔ â·x + b̂ ≥ d

                对于所有面都要成立，所以 d ≤ â·x + b̂。
                我们要让 d 尽量大，也就是让点 x 离所有面的“最小距离”最大。

                4. 目标函数
                clp = [0, 0, 0, -1]，目标是最小化 -d，也就是最大化 d。

                5. 几何意义
                d 就是点 x 到所有面的“最小有向距离”。
                最大化 d，就是找一个点，使它到所有面的距离都大于等于 d，且 d 尽可能大。
                这就是“最深点”或“最大内切球的球心”。

                6. 总结
                x₃（d）就是“深度”，即点到所有面的最小距离。
                clp = [0,0,0,-1]，目标是最大化深度。
                约束保证点在多面体内，且离所有面至少 d。
                这就是为什么 x₃ 表示深度，-x₃ 是最大深度的线性规划目标
        */
        const double maxdepth = -sdlp::linprog<4>(clp, Alp, blp, xlp);
        if (!(maxdepth > 0.0) || std::isinf(maxdepth)) {
            return false;
        }
        const Eigen::Vector3d interior = xlp.head<3>();

        // Prepare the data for MVIE optimization
        // 6. 为MVIE优化准备数据，将多面体约束变换到以interior为中心的坐标系
        // 为LBFGS优化器准备一块连续内存 optData，用于存放所有需要传递的数据。
        // FIXME: 这里的内存分配方式可能会导致内存泄漏，建议使用智能指针或其他内存管理方式。
        uint8_t *optData = new uint8_t[sizeof(int64_t) + (2 + 3 * M) * sizeof(double)];
        // 数据布局如下：
        //     [0]         : int64_t M         // 多面体面数
        //     [1]         : double smoothEps  // 平滑L1惩罚的epsilon参数
        //     [2]         : double penaltyWt  // 约束惩罚权重
        //     [3 ... ]    : double A[M*3]     // 约束矩阵A，M行3列，存储所有面的法向量（已归一化并平移到interior为原点）
        int64_t *pM = (int64_t *) optData;
        double *pSmoothEps = (double *) (pM + 1);
        double *pPenaltyWt = pSmoothEps + 1;
        double *pA = pPenaltyWt + 1;

        // 设置面数M
        *pM = M;

        // 用Eigen::Map将pA映射为Eigen矩阵A，方便后续数值操作
        Eigen::Map<Eigen::MatrixX3d> A(pA, M, 3);
        // 计算每个面的归一化法向量，并将所有面的约束平移到interior为原点
        //   将 A^T x + b <= 0 转为 A'^T x' <= 1 的形式，'表示 interior 为原点的坐标系
        //   A^T x + b <= 0, x = x' + interior
        //   -> A^T(x' + interior) + b <= 0
        //   -> A^T x' + (A^T interior + b) <= 0
        //   -> A^T x' <= -b - A^T interior
        //   -> A^T / (-b - A^T interior) x' <= 1
        //   (blp = -b) -> A^T / (blp - A^T interior) x' <= 1
        //   (Alp = A) -> A' = Alp / (blp - A^T interior)
        A = Alp.leftCols<3>().array().colwise() /
            (blp - Alp.leftCols<3>() * interior).array();

        // 7. 构造优化变量x（9维：中心偏移、Cholesky分解参数）
        Eigen::VectorXd x(9);
        // 生成一个满足 (x-c)^T Q^{-1} (x-c) <= 1 的椭球标准表达
        const Eigen::Matrix3d Q = R * (r.cwiseProduct(r)).asDiagonal() * R.transpose();
        Eigen::Matrix3d L;
        // 计算Q的Cholesky分解，得到下三角矩阵L
        // 这里的L满足 L * L^T = Q
        chol3d(Q, L);

        // 椭球中心在 interior 坐标系下的表达
        x.head<3>() = p - interior;
        // Cholesky分解主对角线
        x(3) = sqrt(L(0, 0));
        x(4) = sqrt(L(1, 1));
        x(5) = sqrt(L(2, 2));
        // Cholesky分解下三角
        x(6) = L(1, 0);
        x(7) = L(2, 1);
        x(8) = L(2, 0);

        // 8. 设置LBFGS优化参数
        double minCost;
        lbfgs::lbfgs_parameter_t paramsMVIE;
        paramsMVIE.mem_size = 18;
        paramsMVIE.g_epsilon = 0.0;
        paramsMVIE.min_step = 1.0e-32;
        paramsMVIE.past = 3;
        paramsMVIE.delta = 1.0e-2;
        *pSmoothEps = 1.0e-2;
        *pPenaltyWt = 1.0e+3;

        // 9. 使用LBFGS优化器最小化目标函数，得到最大体积内接椭球参数
        int ret = lbfgs::lbfgs_optimize(x,
                                        minCost,
                                        &costMVIE,
                                        nullptr,
                                        nullptr,
                                        optData,
                                        paramsMVIE);

        if (ret < 0) {
            printf("FIRI WARNING: %s\n", lbfgs::lbfgs_strerror(ret));
        }

        // 10. 从优化结果x中恢复椭球参数
        p = x.head<3>() + interior;
        L(0, 0) = x(3) * x(3);
        L(0, 1) = 0.0;
        L(0, 2) = 0.0;
        L(1, 0) = x(6);
        L(1, 1) = x(4) * x(4);
        L(1, 2) = 0.0;
        L(2, 0) = x(8);
        L(2, 1) = x(7);
        L(2, 2) = x(5) * x(5);

        // 11. 奇异值分解，提取椭球的旋转矩阵R和半轴长度r
        Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::FullPivHouseholderQRPreconditioner> svd(L, Eigen::ComputeFullU);
        const Eigen::Matrix3d U = svd.matrixU();
        const Eigen::Vector3d S = svd.singularValues();
        if (U.determinant() < 0.0) {
            // 若行列式为负，交换前两列，保证右手系
            R.col(0) = U.col(1);
            R.col(1) = U.col(0);
            R.col(2) = U.col(2);
            r(0) = S(1);
            r(1) = S(0);
            r(2) = S(2);
        } else {
            R = U;
            r = S;
        }

        // 12. 用优化结果更新输出椭球
        ellipsoid = Ellipsoid(R, r, p);
        delete[] optData;
        return ret >= 0;
    }
}
