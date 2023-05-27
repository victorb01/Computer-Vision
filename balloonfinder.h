#ifndef __BALLOON_FINDER_H
#define __BALLOON_FINDER_H

#include <Eigen/Dense>
#include <cstdlib>
#include <memory>
#include <opencv2/opencv.hpp>

#include "sensorparams.h"
#include "structurecomputer.h"

class BalloonFinder {
 public:
  enum class BalloonColor { RED, BLUE };
  // Constructor: gets called when an instance of BalloonFinder is created
  BalloonFinder(bool debuggingEnabled, bool calibrationEnabled,
                const Eigen::Vector3d& blueTrue_I,
                const Eigen::Vector3d& redTrue_I);
  // Destructor: gets called when an instance of BalloonFinder is destructed
  // (destroyed)
  ~BalloonFinder() {}
  // Takes in a pointer to an image and outputs a pointer to an N-by-1 vector
  // of CameraBundle objects, one for each balloon found.  N = bundles->size()
  // indicates the number of balloons found.  Also outputs an N-by-1 vector of
  // colors corresponding to the balloons found.  Thus, if two balloons were
  // found in an image, N = bundles->size() = colors->size() = 2.  The input
  // RCI is the 3x3 attitude matrix relating the C and I frames at the instant
  // the input image was taken.  The input rc_I is the 3x1 position of the
  // camera center at the instant the image was taken, expressed in the I
  // frame in meters.  RCI and rc_I are input so that the corresponding values
  // in each output CameraBundle can be populated.
  void findBalloons(const cv::Mat* image, const Eigen::Matrix3d RCI,
                    const Eigen::Vector3d rc_I,
                    std::vector<std::shared_ptr<const CameraBundle>>* bundles,
                    std::vector<BalloonColor>* colors);
  // Takes in a pointer to an image and a specified balloon color and returns
  // true if a balloon or multiple balloons of that color are found in the
  // image.  The centers of the found balloons are returned in rxVec, in
  // pixels. Note that the coordinates in rxVec are expressed in the camera's
  // image plane coordinate system, whose origin is located at the lower-right
  // of the image, whose x axis points *left* as seen from the camera center
  // out through the image plane, and whose y axis points up.  Returns false
  // if no balloons of the specified color were found.  RCI and rc_I are input
  // for debugging and calibration purposes.  The input RCI is the 3x3
  // attitude matrix relating the C and I frames at the instant the input
  // image was taken.  The input rc_I is the 3x1 position of the camera center
  // at the instant the image was taken, expressed in the I frame in meters.
  bool findBalloonsOfSpecifiedColor(const cv::Mat* image,
                                    const Eigen::Matrix3d RCI,
                                    const Eigen::Vector3d rc_I,
                                    const BalloonFinder::BalloonColor color,
                                    std::vector<Eigen::Vector2d>* rxVec);
  // Returns the updated eCB 312 Euler angles relating the C and B frames
  // after extrinsic camera calibration.  Let the 3x3 matrix RCB be the
  // assumed B-to-C attitude matrix in the SensorParams object.  Then the
  // calibrated matrix RCB_calibrated = dRCB*RCB.  eCB_calibrated holds the
  // Euler angles corresponding to RCB_calibrated.
  Eigen::Vector3d eCB_calibrated() const;

 private:
  SensorParams sensorParams_;
  // Indicates whether interactive debugging is enabled
  bool debuggingEnabled_;
  // Indicates whether camera extrinsic calibratio is enabled
  bool calibrationEnabled_;
  // True location of center of blue balloon in I frame in meters
  Eigen::Vector3d blueTrue_I_;
  // True location of center of red balloon in I frame in meters
  Eigen::Vector3d redTrue_I_;
  // Matrices used in determining the calibration matrix dRCB: V_ holds as
  // columns the unit vectors in the C frame derived from the actual locations
  // of features on the image plane, whereas W_ holds the unit vectors in the
  // C frame derived from back-projecting the known 3d coordinates
  // corresponding to the features.  These are related to dRCB by V_ =
  // dRCB*W_.
  Eigen::MatrixXd V_, W_;
};
#endif
