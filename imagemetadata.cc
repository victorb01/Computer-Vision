#include "imagemetadata.h"

#include "navtoolbox.h"

ImageMetadata::ImageMetadata(std::string line) {
  std::stringstream lineStream(line);
  lineStream >> filename_;
  lineStream >> tImage_.first;
  lineStream >> tImage_.second;
  lineStream >> tPose_.first;
  lineStream >> tPose_.second;
  double temp;
  for (size_t ii = 0; ii < 3; ii++) {
    lineStream >> temp;
    rP_I_(ii) = temp;
  }
  // The image metadata's quaternion relates the antenna plate frame (A) to
  // the local ENU frame (I).  It is formatted according to the ROS
  // convention: qRos = [W, X, Y, Z].  The code below converts this into the
  // internal convention: q = [X, Y, Z, W].
  Eigen::Matrix<double, 4, 1> qRos;
  for (size_t ii = 0; ii < 4; ii++) {
    lineStream >> temp;
    qRos(ii) = temp;
  }
  q_AI_(0) = qRos(1);
  q_AI_(1) = qRos(2);
  q_AI_(2) = qRos(3);
  q_AI_(3) = qRos(0);
}

Eigen::Matrix3d ImageMetadata::RAI() const { return navtbx::quat2dc(q_AI_); }

Eigen::Matrix3d ImageMetadata::RBI() const {
  Eigen::Matrix3d RAB = navtbx::euler2dc(sensorParams_.eAB());
  return RAB.transpose() * RAI();
}

Eigen::Vector3d ImageMetadata::rc_I() const {
  using namespace Eigen;
  // Vector pointing from primary antenna to camera center, expressed in B
  Vector3d P2c_B = sensorParams_.rc() - sensorParams_.rA().col(0);
  // Transform P2c_B to I frame
  Vector3d P2c_I = RBI().transpose() * P2c_B;
  // Sum with rP_I_ to obtain rc_I
  return rP_I_ + P2c_I;
}

Eigen::Matrix3d ImageMetadata::RCI() const {
  Eigen::Matrix3d RCB = navtbx::euler2dc(sensorParams_.eCB());
  return RCB * RBI();
}
