#include "structurecomputer.h"

#include <Eigen/LU>

void pr(Eigen::MatrixXd m) { std::cout << m << std::endl; }
void pr(Eigen::VectorXd m) { std::cout << m << std::endl; }
void pr(Eigen::Matrix3d m) { std::cout << m << std::endl; }
void pr(Eigen::Vector3d m) { std::cout << m << std::endl; }
void pr(Eigen::Vector2d m) { std::cout << m << std::endl; }

Eigen::Vector2d backProject(const Eigen::Matrix3d& RCI,
                            const Eigen::Vector3d& rc_I,
                            const Eigen::Vector3d& X3d) {
  using namespace Eigen;
  Vector3d t = -RCI * rc_I;
  MatrixXd Pextrinsic(3, 4);
  Pextrinsic << RCI, t;
  SensorParams sp;
  MatrixXd Pc = sp.K() * Pextrinsic;
  VectorXd X(4, 1);
  X.head(3) = X3d;
  X(3) = 1;
  Vector3d x = Pc * X;
  Vector2d xc_pixels = (x.head(2) / x(2)) / sp.pixelSize();
  return xc_pixels;
}

Eigen::Vector3d pixelsToUnitVector_C(const Eigen::Vector2d& rPixels) {
  using namespace Eigen;
  SensorParams sp;
  // Convert input vector to meters
  Vector2d rMeters = rPixels * sp.pixelSize();
  // Write as a homogeneous vector, with a 1 in 3rd element
  Vector3d rHomogeneous;
  rHomogeneous.head(2) = rMeters;
  rHomogeneous(2) = 1;
  // Invert the projection operation through the camera intrinsic matrix K to
  // yield a vector rC in the camera coordinate frame that has a Z value of 1
  Vector3d rC = sp.K().lu().solve(rHomogeneous);
  // Normalize rC so that output is a unit vector
  return rC.normalized();
}

void StructureComputer::clear() {
  // Zero out contents of point_
  point_.rXIHat.fill(0);
  point_.Px.fill(0);
  // Clear bundleVec_
  bundleVec_.clear();
}

void StructureComputer::push(std::shared_ptr<const CameraBundle> bundle) {
  bundleVec_.push_back(bundle);
}

// This function is where the computation is performed to estimate the
// contents of point_.  The function returns a copy of point_.
//
Point StructureComputer::computeStructure() {
  // Throw an error if there are fewer than 2 CameraBundles in bundleVec_,
  // since in this case structure computation is not possible.
  if (bundleVec_.size() < 2) {
    throw std::runtime_error(
        "At least 2 CameraBundle objects are "
        "needed for structure computation.");
  }

  SensorParams sp;
  std::shared_ptr<const CameraBundle> bundle;
  int N = static_cast<int>(bundleVec_.size());
  Eigen::MatrixXd eq1(1,4), eq2(1,4);
  Eigen::MatrixXd H(2*N, 4);
  Eigen::MatrixXd R(2*N, 2*N);
  R.fill(0);

  for (int i = 0; i < N; i++)  {
    bundle = bundleVec_[i];
    Eigen::Vector2d rxVec = sp.pixelSize() * bundle->rx;
    Eigen::Matrix3d RCI = bundle->RCI;
    Eigen::Vector3d rcVec = bundle->rc_I;
    //Create projection matrix P
    Eigen::Vector3d t = -RCI * rcVec;
    Eigen::MatrixXd Pextrinsic(3, 4);
    Pextrinsic << RCI, t;
    Pextrinsic = sp.K()*Pextrinsic;
    //Create H matrix and R matrix
    eq1 = rxVec(0)*Pextrinsic.block(2,0,1,4) - Pextrinsic.block(0,0,1,4);
    eq2 = rxVec(1)*Pextrinsic.block(2,0,1,4) - Pextrinsic.block(1,0,1,4);
    H.block(2*i,0,1,4) = eq1;
    H.block(2*i+1,0,1,4) = eq2;
    R.block(2*i,2*i,2,2) = sp.pixelSize()*sp.pixelSize()*sp.Rc();
  }
  //Extract Hr and z from H
  Eigen::MatrixXd Hr(2*N,3);
  Hr = H.block(0,0,2*N,3);
  Eigen::MatrixXd z(2*N,1);
  z = -H.block(0,3,2*N,1);
  //Create rXIHat and Px
  Eigen::MatrixXd Rinv(2*N,2*N);
  Rinv = R.inverse();
  point_.rXIHat = (Hr.transpose() * Rinv * Hr).ldlt().solve(Hr.transpose() * Rinv * z);
  point_.Px = (Hr.transpose() * Rinv * Hr).inverse();
  return point_;
}
