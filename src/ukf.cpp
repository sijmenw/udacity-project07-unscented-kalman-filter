#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // TODO properly estimate this
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  is_initialized_ = false;

  // from 5.26
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_x_;

  weights_ = VectorXd(2 * n_aug_ + 1);

  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i = 1; i < 2*n_aug_+1; i++) {
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_(i) = weight;
  }
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  if (!is_initialized_) {
      // initialize
      if (meas_package.sensor_type_ == meas_package.LASER) {
          cout << "Initializing with laser package!\n";

          // extract LASER values
          double px = meas_package.raw_measurements_[0];
          double py = meas_package.raw_measurements_[1];

          // initialize state x
          x_ << px, py, 2, 0, 0;

          // initialize state covariance matrix P with laser values
          P_ << std_laspx_ * std_laspx_, 0, 0, 0, 0,
                0, std_laspy_ * std_laspy_, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1;

      } else if (meas_package.sensor_type_ == meas_package.RADAR) {
          cout << "Initializing with radar package!\n";

          // extract RADAR values
          double phi = meas_package.raw_measurements_[0];
          double rho = meas_package.raw_measurements_[1];
          double rhodot = meas_package.raw_measurements_[2];

          x_ << cos(phi) * rho, sin(phi) * rho, 2, 0, 0;

          // initialize state covariance matrix P with laser values
          P_ << std_radr_ * std_radr_, 0, 0, 0, 0,
                0, std_radr_ * std_radr_, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, std_radphi_ * std_radphi_, 0,
                0, 0, 0, 0, 1;
      } else {
          cout << "WARNING: NOT INITIALIZED!\n";
      }

      time_us_ = meas_package.timestamp_;
      is_initialized_ = true;

      // don't process the package used to initialize
      return;
  }

  // calculate delta t
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  // predict step
  Prediction(dt);

  // update step
  if (meas_package.sensor_type_ == meas_package.LASER) {
      UpdateLidar(meas_package);
  } else if (meas_package.sensor_type_ == meas_package.RADAR) {
      UpdateRadar(meas_package);
  }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
