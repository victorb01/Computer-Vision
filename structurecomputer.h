#ifndef __STRUCTURE_COMPUTER_H
#define __STRUCTURE_COMPUTER_H

#include <Eigen/Dense>
#include <cstdlib>
#include <memory>
#include <vector>

#include "sensorparams.h"

struct CameraBundle {
  // 2x1 coordinates of feature point as projected on the camera's image
  // plane, in pixels.  Note that the camera's image plane coordinate system
  // has its origin at the lower right corner of the image, its x axis
  // pointing to the *left* as seen from the camera center out through the
  // image plane, its z axis pointing along the boresight of the camera, and
  // its y axis pointing up, completing the right-handed triad.  This means
  // that x coodinates in the image plane increase to the *left* and y
  // coordinates increase *up*.
  Eigen::Vector2d rx;
  // 3x3 attitude matrix relating the C and I frames at the instant the image
  // was taken.
  Eigen::Matrix3d RCI;
  // 3x1 position of the camera center at the instant the image was taken,
  // expressed in the I frame in meters.
  Eigen::Vector3d rc_I;
  // Macro that ensures proper aligment of Eigen objects in struct
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Point {
  // 3x1 estimated location of the feature point expressed in I in meters
  Eigen::Vector3d rXIHat;
  // 3x3 error covariance matrix for the estimate rxIHat
  Eigen::Matrix3d Px;
  // Macro that ensures proper aligment of Eigen objects in struct
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class StructureComputer {
 public:
  // Constructor: gets called when an instance of StructureComputer is created
  StructureComputer() {}
  // Destructor: gets called when an instance of StructureComputer is destroyed
  ~StructureComputer() {}
  // Clears out contents; puts StructureComputer instance in initial state
  void clear();
  // Add another CameraBundle object to those from which structure will be
  // computed
  void push(std::shared_ptr<const CameraBundle> bundle);
  // Computes a Point from all CameraBundle objects that have been pushed
  Point computeStructure();
  // Returns a copy of the stored Point object
  Point point() { return point_; }

 private:
  SensorParams sensorParams_;
  // The Point object estimated from the CameraBundle objects in bundleVec_
  Point point_;
  // Vector of pointers to CameraBundle objects all corresponding to the same
  // 3d feature
  std::vector<std::shared_ptr<const CameraBundle>> bundleVec_;
};

// Back-projects the 3D point X3d (expressed in the I frame) onto the image
// plane of the camera whose attitude and position with respect to the I frame
// are specified by RCI and rc_I, respectively.  Returns a 2-by-1 vector
// projection of X3d onto the image plane, expressed in pixels.
//
Eigen::Vector2d backProject(const Eigen::Matrix3d& RCI,
                            const Eigen::Vector3d& rc_I,
                            const Eigen::Vector3d& X3d);

// Returns the unit vector, in the camera frame, pointing from the camera
// origin through the coordinate rPixels (expressed in pixels) on the camera's
// image plane.
//
Eigen::Vector3d pixelsToUnitVector_C(const Eigen::Vector2d& rPixels);

// Functions for easy printing in gdb
void pr(Eigen::MatrixXd m);
void pr(Eigen::VectorXd m);
void pr(Eigen::Matrix3d m);
void pr(Eigen::Vector3d m);
void pr(Eigen::Vector2d m);

#endif
