#include <jpcc/process/JPCCNormalEstimation.h>

#include <execution>
#include <utility>

#include <Eigen/Dense>

#include <KDTreeVectorOfVectorsAdaptor.h>

namespace jpcc::process {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCNormalEstimation::JPCCNormalEstimation(JPCCNormalEstimationParameter param) : param_(std::move(param)) {
  if (!param_.enable) { BOOST_THROW_EXCEPTION(std::logic_error(std::string("Normal Estimation not enabled"))); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCNormalEstimation::computeInPlace(FramePtr& frame) const {
  frame->addNormals();

  KDTreeVectorOfVectorsAdaptor<Frame, double> kdtree(3, *frame, 10);

  Eigen::Vector3d bary     = Eigen::Vector3d ::Zero();
  Eigen::Vector3d normal   = Eigen::Vector3d ::Zero();
  Eigen::Vector3d eigenval = Eigen::Vector3d ::Zero();
  Eigen::Matrix3d covMat;
  Eigen::Matrix3d Q;
  Eigen::Matrix3d D;
  for (size_t index = 0; index < frame->getPointCount(); index++) {
    auto&                 point       = (*frame)[index];
    std::array<double, 3> pointDouble = {(double)point[0], (double)point[1], (double)point[2]};
    std::vector<size_t>   indices;
    if (param_.radiusSearch > 0) {
      std::vector<std::pair<Index, double> >    indicesDists;
      nanoflann::RadiusResultSet<double, Index> resultSet(param_.radiusSearch * param_.radiusSearch, indicesDists);
      resultSet.init();
      bool ret = kdtree.index->radiusSearchCustomCallback(&pointDouble[0], resultSet, nanoflann::SearchParams(10));
      THROW_IF_NOT(ret);
      indices.clear();
      for (const auto& [ii, dd] : indicesDists) { indices.push_back(ii); }
    } else if (param_.kSearch > 0) {
      size_t                          K = param_.kSearch;
      std::vector<size_t>             _indices(K);
      std::vector<double>             distancies(K);
      nanoflann::KNNResultSet<double> resultSet(K);
      resultSet.init(&_indices[0], &distancies[0]);
      bool ret = kdtree.index->findNeighbors(resultSet, &pointDouble[0], nanoflann::SearchParams(10));
      THROW_IF_NOT(ret);
      indices = _indices;
    }
    THROW_IF_NOT(indices.size() > 1);

    {
      for (auto neighborIndex : indices) {
        auto& neighborPoint = (*frame)[neighborIndex];
        bary += Eigen::Vector3d(neighborPoint[0], neighborPoint[1], neighborPoint[2]);
      }
      bary /= double(indices.size());
      covMat = Eigen::Matrix3d::Zero();
      Eigen::Vector3d pt;
      for (unsigned long long neighborIndex : indices) {
        auto& neighborPoint = (*frame)[neighborIndex];
        pt                  = Eigen::Vector3d(neighborPoint[0], neighborPoint[1], neighborPoint[2]) - bary;
        covMat(0, 0) += pt[0] * pt[0];
        covMat(1, 1) += pt[1] * pt[1];
        covMat(2, 2) += pt[2] * pt[2];
        covMat(0, 1) += pt[0] * pt[1];
        covMat(0, 2) += pt[0] * pt[2];
        covMat(1, 2) += pt[1] * pt[2];
      }
      covMat(1, 0) = covMat(0, 1);
      covMat(2, 0) = covMat(0, 2);
      covMat(2, 1) = covMat(1, 2);
      covMat /= (double(indices.size()) - 1.0);

      PCCDiagonalize(covMat, Q, D);

      D(0, 0) = fabs(D(0, 0));
      D(1, 1) = fabs(D(1, 1));
      D(2, 2) = fabs(D(2, 2));

      if (D(0, 0) < D(1, 1) && D(0, 0) < D(2, 2)) {
        normal[0]   = Q(0, 0);
        normal[1]   = Q(1, 0);
        normal[2]   = Q(2, 0);
        eigenval[0] = D(0, 0);
        if (D(1, 1) < D(2, 2)) {
          eigenval[1] = D(1, 1);
          eigenval[2] = D(2, 2);
        } else {
          eigenval[2] = D(1, 1);
          eigenval[1] = D(2, 2);
        }
      } else if (D(1, 1) < D(2, 2)) {
        normal[0]   = Q(0, 1);
        normal[1]   = Q(1, 1);
        normal[2]   = Q(2, 1);
        eigenval[0] = D(1, 1);
        if (D(0, 0) < D(2, 2)) {
          eigenval[1] = D(0, 0);
          eigenval[2] = D(2, 2);
        } else {
          eigenval[2] = D(0, 0);
          eigenval[1] = D(2, 2);
        }
      } else {
        normal[0]   = Q(0, 2);
        normal[1]   = Q(1, 2);
        normal[2]   = Q(2, 2);
        eigenval[0] = D(2, 2);
        if (D(0, 0) < D(1, 1)) {
          eigenval[1] = D(0, 0);
          eigenval[2] = D(1, 1);
        } else {
          eigenval[2] = D(0, 0);
          eigenval[1] = D(1, 1);
        }
      }
    }

    frame->getNormal(index)[0] = float(normal[0]);
    frame->getNormal(index)[1] = float(normal[1]);
    frame->getNormal(index)[2] = float(normal[2]);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCNormalEstimation::computeInPlaceAll(GroupOfFrame& frames, bool parallel) const {
  if (!parallel) {
    for (auto& frame : frames) { this->computeInPlace(frame); }
  } else {
    std::for_each(std::execution::par_unseq, frames.begin(), frames.end(),
                  [this](FramePtr& frame) { this->computeInPlace(frame); });
  }
}

// Slightly modified version of http://www.melax.com/diag.html?attredirects=0
// A must be a symmetric matrix.
// returns Q and D such that
// Diagonal matrix D = QT * A * Q;  and  A = Q*D*QT
void PCCDiagonalize(const Eigen::Matrix3d& A, Eigen::Matrix3d& Q, Eigen::Matrix3d& D) {
  const int       maxsteps = 24;  // certainly wont need that many.
  int             k0, k1, k2;
  double          o[3], m[3];
  double          q[4] = {0.0, 0.0, 0.0, 1.0};
  double          jr[4];
  double          sqw, sqx, sqy, sqz;
  double          tmp1, tmp2, mq;
  Eigen::Matrix3d AQ;
  double          thet, sgn, t, c;
  for (int i = 0; i < maxsteps; ++i) {
    // quat to matrix
    sqx     = q[0] * q[0];
    sqy     = q[1] * q[1];
    sqz     = q[2] * q[2];
    sqw     = q[3] * q[3];
    Q(0, 0) = (sqx - sqy - sqz + sqw);
    Q(1, 1) = (-sqx + sqy - sqz + sqw);
    Q(2, 2) = (-sqx - sqy + sqz + sqw);
    tmp1    = q[0] * q[1];
    tmp2    = q[2] * q[3];
    Q(1, 0) = 2.0 * (tmp1 + tmp2);
    Q(0, 1) = 2.0 * (tmp1 - tmp2);
    tmp1    = q[0] * q[2];
    tmp2    = q[1] * q[3];
    Q(2, 0) = 2.0 * (tmp1 - tmp2);
    Q(0, 2) = 2.0 * (tmp1 + tmp2);
    tmp1    = q[1] * q[2];
    tmp2    = q[0] * q[3];
    Q(2, 1) = 2.0 * (tmp1 + tmp2);
    Q(1, 2) = 2.0 * (tmp1 - tmp2);

    // AQ = A * Q;
    AQ(0, 0) = Q(0, 0) * A(0, 0) + Q(1, 0) * A(0, 1) + Q(2, 0) * A(0, 2);
    AQ(0, 1) = Q(0, 1) * A(0, 0) + Q(1, 1) * A(0, 1) + Q(2, 1) * A(0, 2);
    AQ(0, 2) = Q(0, 2) * A(0, 0) + Q(1, 2) * A(0, 1) + Q(2, 2) * A(0, 2);
    AQ(1, 0) = Q(0, 0) * A(0, 1) + Q(1, 0) * A(1, 1) + Q(2, 0) * A(1, 2);
    AQ(1, 1) = Q(0, 1) * A(0, 1) + Q(1, 1) * A(1, 1) + Q(2, 1) * A(1, 2);
    AQ(1, 2) = Q(0, 2) * A(0, 1) + Q(1, 2) * A(1, 1) + Q(2, 2) * A(1, 2);
    AQ(2, 0) = Q(0, 0) * A(0, 2) + Q(1, 0) * A(1, 2) + Q(2, 0) * A(2, 2);
    AQ(2, 1) = Q(0, 1) * A(0, 2) + Q(1, 1) * A(1, 2) + Q(2, 1) * A(2, 2);
    AQ(2, 2) = Q(0, 2) * A(0, 2) + Q(1, 2) * A(1, 2) + Q(2, 2) * A(2, 2);

    // D  = Q.transpose() * AQ;
    D(0, 0) = AQ(0, 0) * Q(0, 0) + AQ(1, 0) * Q(1, 0) + AQ(2, 0) * Q(2, 0);
    D(0, 1) = AQ(0, 0) * Q(0, 1) + AQ(1, 0) * Q(1, 1) + AQ(2, 0) * Q(2, 1);
    D(0, 2) = AQ(0, 0) * Q(0, 2) + AQ(1, 0) * Q(1, 2) + AQ(2, 0) * Q(2, 2);
    D(1, 0) = AQ(0, 1) * Q(0, 0) + AQ(1, 1) * Q(1, 0) + AQ(2, 1) * Q(2, 0);
    D(1, 1) = AQ(0, 1) * Q(0, 1) + AQ(1, 1) * Q(1, 1) + AQ(2, 1) * Q(2, 1);
    D(1, 2) = AQ(0, 1) * Q(0, 2) + AQ(1, 1) * Q(1, 2) + AQ(2, 1) * Q(2, 2);
    D(2, 0) = AQ(0, 2) * Q(0, 0) + AQ(1, 2) * Q(1, 0) + AQ(2, 2) * Q(2, 0);
    D(2, 1) = AQ(0, 2) * Q(0, 1) + AQ(1, 2) * Q(1, 1) + AQ(2, 2) * Q(2, 1);
    D(2, 2) = AQ(0, 2) * Q(0, 2) + AQ(1, 2) * Q(1, 2) + AQ(2, 2) * Q(2, 2);

    o[0] = D(1, 2);
    o[1] = D(0, 2);
    o[2] = D(0, 1);
    m[0] = fabs(o[0]);
    m[1] = fabs(o[1]);
    m[2] = fabs(o[2]);

    k0 = (m[0] > m[1] && m[0] > m[2]) ? 0 : (m[1] > m[2]) ? 1 : 2;  // index of largest element of offdiag
    k1 = (k0 + 1) % 3;
    k2 = (k0 + 2) % 3;
    if (o[k0] == 0.0) {
      break;  // diagonal already
    }
    thet = (D(k2, k2) - D(k1, k1)) / (2.0 * o[k0]);
    sgn  = (thet > 0.0) ? 1.0 : -1.0;
    thet *= sgn;                                                          // make it positive
    t = sgn / (thet + ((thet < 1.E6) ? sqrt(thet * thet + 1.0) : thet));  // sign(T)/(|T|+sqrt(T^2+1))
    c = 1.0 / sqrt(t * t + 1.0);                                          //  c= 1/(t^2+1) , t=s/c
    if (c == 1.0) {
      break;  // no room for improvement - reached machine precision.
    }
    jr[0] = jr[1] = jr[2] = jr[3] = 0.0;
    jr[k0]                        = sgn * sqrt((1.0 - c) / 2.0);  // using 1/2 angle identity sin(a/2)
                                                                  // = sqrt((1-cos(a))/2)
    jr[k0] *= -1.0;  // since our quat-to-matrix convention was for v*M instead of M*v
    jr[3] = sqrt(1.0 - jr[k0] * jr[k0]);
    if (jr[3] == 1.0) {
      break;  // reached limits of floating point precision
    }
    q[0] = (q[3] * jr[0] + q[0] * jr[3] + q[1] * jr[2] - q[2] * jr[1]);
    q[1] = (q[3] * jr[1] - q[0] * jr[2] + q[1] * jr[3] + q[2] * jr[0]);
    q[2] = (q[3] * jr[2] + q[0] * jr[1] - q[1] * jr[0] + q[2] * jr[3]);
    q[3] = (q[3] * jr[3] - q[0] * jr[0] - q[1] * jr[1] - q[2] * jr[2]);
    mq   = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] /= mq;
    q[1] /= mq;
    q[2] /= mq;
    q[3] /= mq;
  }
}
}  // namespace jpcc::process