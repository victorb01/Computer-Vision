#ifndef __SENSOR_PARAMS_H
#define __SENSOR_PARAMS_H

#include <Eigen/Dense>
#include <iostream>

#include "definitions.h"

class SensorParams {
 public:
  SensorParams();
  Eigen::Matrix<double, 3, 2> rA() const { return rA_; }
  Eigen::Vector3d eAB() const { return eAB_; }
  Eigen::Vector3d rc() const { return rc_; }
  Eigen::Vector3d eCB() const { return eCB_; }
  double pixelSize() const { return pixelSize_; }
  double f() const { return f_; }
  size_t imageWidthPixels() const { return imageWidthPixels_; }
  size_t imageHeightPixels() const { return imageHeightPixels_; }
  double cx() const { return cx_; }
  double cy() const { return cy_; }
  Eigen::Matrix3d K() const { return K_; }
  Eigen::Matrix<double, 5, 1> distortionCoeffs() const {
    return distortionCoeffs_;
  }
  Eigen::Matrix2d Rc() const { return Rc_; }

 private:
  //--------------------- GNSS --------------------------------
  // rA.col(0) holds the 3x1 position of the primary GNSS antenna in the body
  // frame, in meters.  rA.col(1) holds the position of the secondary antenna.
  Eigen::Matrix<double, 3, 2> rA_;
  // 312 Euler angles relating the body and antenna plate reference frames, in
  // radians.  The antenna plate reference frame (A) is defined as follows: x
  // axis: points from the primary to the secondary antenna, z axis: points
  // along the boresight of the antennas, y axis: completes the right-handed
  // triad.  The body frame (B) is the same except that its x axis points from
  // the secondary to the primary antenna.
  Eigen::Vector3d eAB_;

  //--------------------- HD Camera ---------------------------
  // 3x1 position of the HD camera center in the body frame, in meters.
  Eigen::Vector3d rc_;
  // 312 Euler angles relating the HD camera and body reference
  // frames, in radians.
  Eigen::Vector3d eCB_;
  // Pixel size, in meters
  double pixelSize_;
  // Distance of the image plane along camera z axis, in meters
  double f_;
  // 4k image width and height in pixels
  size_t imageWidthPixels_, imageHeightPixels_;
  // Camera principal point, in meters
  double cx_, cy_;
  // Camera intrinsic matrix. Note that all quantities in K have units of
  // meters. Divide f and c values by pixelSize to express in pixels.
  Eigen::Matrix3d K_;
  // Camera distortion coefficients in this order: k1, k2, p1, p2, k3
  Eigen::Matrix<double, 5, 1> distortionCoeffs_;
  // 2x2 error covariance matrix for the Gaussian image noise, in pixels^2
  Eigen::Matrix2d Rc_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
