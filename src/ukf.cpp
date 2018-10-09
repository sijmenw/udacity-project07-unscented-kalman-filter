#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

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

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3.5/2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI/4;

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

  // from 7.26
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

  // sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  time_us_ = 0;

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

  // skip types that are not used
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && !use_laser_) {
      return;
  }
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && !use_radar_) {
      return;
  }

  if (!is_initialized_) {
      // initialize
      if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
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

      } else {
          cout << "Initializing with radar package!\n";

          // extract RADAR values
          double phi = meas_package.raw_measurements_[0];
          double rho = meas_package.raw_measurements_[1];

          x_ << cos(phi) * rho, sin(phi) * rho, 2, 0, 0;

          // initialize state covariance matrix P with laser values
          P_ << std_radr_ * std_radr_, 0, 0, 0, 0,
                0, std_radr_ * std_radr_, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, std_radphi_ * std_radphi_, 0,
                0, 0, 0, 0, 1;
      }

      time_us_ = meas_package.timestamp_;
      is_initialized_ = true;

      // don't process the package used to initialize
      cout << "Done initializing\n";
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

  // generate sigma points
  // from 7.15

  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();

  //set first column of sigma point matrix
  Xsig.col(0)  = x_;

  //set remaining sigma points
  for (int i = 0; i < n_x_; i++) {
    Xsig.col(i + 1)         = x_ + sqrt(lambda_ + n_x_) * A.col(i);
    Xsig.col(i + 1 + n_x_)  = x_ - sqrt(lambda_ + n_x_) * A.col(i);
  }

  // from 7.17
  // init augmented state
  VectorXd x_aug_ = VectorXd(7);
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug_.col(0) = x_aug_;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug_.col(i + 1) = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  // predict sigma points
  // from 7.20
  //predict sigma points
    //predict sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        float dt = delta_t;
        float dt2 = dt * dt;
        float pxo = Xsig_aug_.col(i)[0];
        float pyo = Xsig_aug_.col(i)[1];
        float vo = Xsig_aug_.col(i)[2];
        float yo = Xsig_aug_.col(i)[3];
        float yro = Xsig_aug_.col(i)[4];
        float vao = Xsig_aug_.col(i)[5];
        float yrao = Xsig_aug_.col(i)[6];

        // matrix 1
        //avoid division by zero
        float px;
        float py;
        if (yro != 0) {
            px = pxo + vo / yro * (sin(yo + yro * dt) - sin(yo));
            py = pyo + vo / yro * (-cos(yo + yro * dt) + cos(yo));
        } else {
            px = pxo + vo * cos(yo) * dt;
            py = pyo + vo * sin(yo) * dt;
        }

        float v = vo + 0;
        float y = yo + yro * dt;
        float yr = yro + 0;

        // add noise processes
        px += 0.5 * dt2 * cos(yo) * vao;
        py += 0.5 * dt2 * sin(yo) * vao;
        v += dt * vao;
        y += 0.5 * dt2 * yrao;
        yr += dt * yrao;


        //write predicted sigma points into right column
        Xsig_pred_.col(i) << px, py, v, y, yr;
    }


  // predict mean and covariance
    //from 7.23
    // set weights
    double weight_0 = lambda_/(lambda_+n_aug_);
    weights_(0) = weight_0;
    for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
        double weight = 0.5/(n_aug_+lambda_);
        weights_(i) = weight;
    }

    //predicted state mean
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        x_ = x_+ weights_(i) * Xsig_pred_.col(i);
    }

    //predicted state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
    }

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
    int n_z = 2; // lidar has 2 measurement dimensions
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);


    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);

        // measurement model
        Zsig(0, i) = p_x; //px
        Zsig(1, i) = p_y;          //py
    }

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    //innovation covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);
    R <<    std_laspx_*std_laspx_, 0,
            0, std_laspy_*std_laspy_;

    S = S + R;

    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);

    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    //residual
    VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();
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

  // from 7.26
  int n_z = 3; // radar has 3 measurement dimensions
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);


    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v  = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);

        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;

        // measurement model
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
        Zsig(1,i) = atan2(p_y,p_x);                                 //phi
        Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
    }

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    //innovation covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R <<    std_radr_*std_radr_, 0, 0,
            0, std_radphi_*std_radphi_, 0,
            0, 0,std_radrd_*std_radrd_;
    S = S + R;

    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);

    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    //residual
    VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();

}
