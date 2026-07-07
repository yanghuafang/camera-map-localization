// Softmax-weighted covariance of the cost surface around the argmin hypothesis.

#include <cam_loc/core/sampling_covariance.hpp>

#include <cmath>

namespace cam_loc::core {

SamplingConfidence SamplingCovariance::compute(const CostGrid& grid,
                                               const CostGrid::ArgMinResult& argmin,
                                               float cost_scale) {
  // Turns the shape of the cost surface into a measurement covariance for the EKF: a sharp,
  // isolated minimum yields a small covariance (trust the map match) while a broad or flat basin
  // yields a large one. It is a softmax-weighted second moment of the offset cloud about the
  // argmin — a cheap confidence proxy, not the true cost Hessian.
  SamplingConfidence conf;
  if (cost_scale <= 0.f) {
    return conf;
  }

  const Vec3 mu = grid.indexToOffset(argmin.ix, argmin.iy, argmin.iw);

  // exp(-(c - c_min) / scale) weights; spread in (x, y, yaw) offset space
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  double weight_sum = 0.0;

  for (int iw = 0; iw < grid.dimW(); ++iw) {
    for (int iy = 0; iy < grid.dimY(); ++iy) {
      for (int ix = 0; ix < grid.dimX(); ++ix) {
        const float c = grid.at(ix, iy, iw);
        const double w = std::exp(-static_cast<double>(c - argmin.cost) / cost_scale);
        if (w < 1e-6) continue;
        const Vec3 x = grid.indexToOffset(ix, iy, iw);
        const Vec3 d = x - mu;
        cov += w * d * d.transpose();
        weight_sum += w;
      }
    }
  }

  if (weight_sum > 1e-9) {
    cov /= weight_sum;
    conf.covariance = cov;
    conf.valid = true;
  }
  return conf;
}

}  // namespace cam_loc::core
