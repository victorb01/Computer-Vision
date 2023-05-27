#ifndef __IMAGE_METADATA_H
#define __IMAGE_METADATA_H

#include <Eigen/Dense>
#include <cstdlib>
#include <memory>

#include "sensorparams.h"

// Stores and manipulates the metadata associated with each image, such as
// camera pose.
class ImageMetadata {
 public:
  // Default constructor
  ImageMetadata(void) {}
  // Constructor that populates data members from a single line of metadata file
  ImageMetadata(std::string line);
  // Returns filename
  std::string filename() const { return filename_; }
  // Returns the 3x1 position of the camera center at the instant the image
  // was taken, expressed in the I frame in meters.
  Eigen::Vector3d rc_I() const;
  // Returns the 3x3 attitude matrix relating the C and I frames at the
  // instant the image was taken.
  Eigen::Matrix3d RCI() const;
  // Returns the 3x3 attitude matrix relating the B and I frames at the
  // instant the image was taken.
  Eigen::Matrix3d RBI() const;
  // Returns the 3x3 attitude matrix relating the A and I frames at the
  // instant the image was taken.
  Eigen::Matrix3d RAI() const;
  Eigen::MatrixXd q_AI() const { return q_AI_; }

 private:
  // Image filename
  std::string filename_;
  // Position of primary antenna in I frame (local ENU frame), in meters
  Eigen::Vector3d rP_I_;
  // Quaternion relating antenna plate frame and I (local ENU) frame
  Eigen::Matrix<double, 4, 1> q_AI_;
  // Time stamp of image in Snapdragon system time: the first element is
  // seconds and the second is nanoseconds.
  std::pair<unsigned int, unsigned int> tImage_;
  // Time stamp of camera pose in Snapdragon system time: the first element is
  // seconds and the second is nanoseconds.
  std::pair<unsigned int, unsigned int> tPose_;
  // Sensor parameters
  SensorParams sensorParams_;
};

#endif
