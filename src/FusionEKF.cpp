#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  long long currTime = measurement_pack.timestamp_;
  long long prevTime = previous_timestamp_;

  previous_timestamp_ = currTime;
  VectorXd measures = measurement_pack.raw_measurements_;

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    ekf_.P_ = MatrixXd(4,4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho   = measures(0);
      float theta = measures(1);
      ekf_.x_ << rho*cos(theta), rho*sin(theta), 0, 0;
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << measures(0), measures(1), 0, 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  const float noise_ax = 9;
  const float noise_ay = 9;

  float dt = (currTime - prevTime) / 1000000.0f;

  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;

  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 = dt3*dt;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt4/4*noise_ax, 0, dt3/2*noise_ax, 0,
              0, dt4/4*noise_ay, 0, dt3/2*noise_ay,
              dt3/2*noise_ax, 0, dt2*noise_ax, 0,
              0, dt3/2*noise_ay, 0, dt2*noise_ay;

  std::cout << "Debug: here!" << std::endl;

  ekf_.Predict();

  std::cout << "Debug: here1!" << std::endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    std::cout << "Debug: here2*!" << std::endl;
    ekf_.UpdateEKF(measures);
    std::cout << "Debug: here2!" << std::endl;
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    std::cout << "Debug: here3*!" << std::endl;
    ekf_.Update(measures);
    std::cout << "Debug: here3!" << std::endl;
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
