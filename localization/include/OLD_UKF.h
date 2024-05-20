#ifndef UKF_H
#define UKF_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <math.h>
#include <tuple>
#include <cmath>
#include "Quaternion.h"

#define PI 3.14159265358979323846

// --- Global Frame Values ---
// Magnetometer
// Constants dervied from location: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
#define INCLINATION -68.5006 * (PI/180.0)      // Inclination Angle (rads) 
#define DECLINATION 11.4017 * (PI/180.0)       // Declination Angle (rads)
#define B_INTENSITY 21951.5e-9                 // Magnetic Field Intensity (Tesla)

// Merwe Scaled Sigma points for UKF
class MerwedSigmaPoints
{
public:
	// Sigma points
	int n;
	int num_sigma_points;

	// Sigma point parameters
	double alpha, beta, kappa;

	// Sigma point Weights 
	Eigen::VectorXd Wm;
	Eigen::VectorXd Wc;

	/*** Constructors ***/
	MerwedSigmaPoints();		// Default Constructor
	MerwedSigmaPoints(int n, double alpha, double beta, double kappa);

	/*** Destructors ***/
	virtual ~MerwedSigmaPoints();

	/*** Weight Computations ***/
	Eigen::VectorXd compute_Wm();
	Eigen::VectorXd compute_Wc();

	/*** Sigma Points ***/
	Eigen::MatrixXd calculate_sigma_points(Eigen::VectorXd mean, Eigen::MatrixXd cov);

};
// Unscented kalman filter for orientation estimation 
class UKF
{
public:
	/*** UKF State Variables ***/
	// State vector: [q0 q1 q2 q3 wx wy wz x y].T
	int x_dim;					// State dimension
	Eigen::VectorXd x_hat;		// Estimated State (mean)
	Eigen::VectorXd x_prior;	// x_state prediction (or x_bar)
	Eigen::VectorXd x_post;		// x_state poseterior

	// Measurement vector: []
	int z_dim;
	Eigen::VectorXd z;			// z_state
	Eigen::VectorXd z_prior;    // z_state prediction (or z_bar)
	Eigen::MatrixXd S;		    // Posteriori measurement covariance matrix

	// Posteriori Estimate Covariance Matrix 
	Eigen::MatrixXd P;			// Posteriori estimate covariance matrix
	Eigen::MatrixXd P_prior;	// Priori prediction cov matrix
	Eigen::MatrixXd P_post;		// Posteriori cov matrix cache

	/*** UKF Sigma Points ***/
	// Sigma points
	MerwedSigmaPoints sigma_points;
	// Eigen::MatrixXd sigma_points;

	// Transformed sigma points (after being passed through f(x) and h(x))
	Eigen::MatrixXd X_sigma;	// Predicted sigma points gamma = f(x,t) (prob book: g(u_t,sigma_t-1))
	Eigen::MatrixXd Z_sigma;	// Measurement sigma points Zbar = h(X_bar)

	/*** Noise Matrices ***/
	// Process noise covariance matrix
	Eigen::MatrixXd Q;

	// Observation noise covariance matrix
	Eigen::MatrixXd R;

	// Inertial frame of gravity and magnetometer values
	// g0 eqn (Robotics Vision and Control p83)
	// m0 eqn (Robotics Vision and Control p85)
	Eigen::Vector3d g0;
	Eigen::Vector3d m0;

	/*** Constructors ***/
	UKF();
	UKF(MerwedSigmaPoints merwed_sigma_points);
	UKF(int x_dim_, int z_dim_, MerwedSigmaPoints merwed_sigma_points);

	/*** Destructors ***/
	virtual ~UKF();

	/*** Position Prediction + Update Steps ***/
	void predict_states(Eigen::VectorXd z_measurement, double dt);
	void predict_measurement(double dt, Eigen::VectorXd w, double lon0, double lat0);

	/*** Orientation Prediction + Update Steps ***/
	void predict_with_quaternion_ang_vec_model(double dt, Eigen::VectorXd u_t);
	void update_with_quaternion_ang_vec_model(Eigen::MatrixXd z_measurement);

	std::tuple<Eigen::VectorXd, Eigen::MatrixXd> unscented_transform(Eigen::MatrixXd sigmas,
		Eigen::MatrixXd Wm,
		Eigen::MatrixXd Wc,
		Eigen::MatrixXd noise_cov);

	/*** Nonlinear functions **/
	Eigen::VectorXd process_model(Eigen::VectorXd x, Eigen::VectorXd z_measurement, double dt);
	Eigen::VectorXd measurment_model(Eigen::VectorXd x, Eigen::VectorXd w, double lon0, double lat0,double dt);

	void update(Eigen::MatrixXd z_measurement);
};

class ROVER
{
public:

	/**** Constructor ****/
	ROVER();

	/**** Destructor ****/
	virtual ~ROVER();

	/*** ROVER Kinematic model ***/
	double a1;
	double a2;
	double a3;
	double a4;

	// ROVER motion
	double velocity;
	double omega;
	double d;

	// velocity model
	void calculate_wheel_change(Eigen::VectorXd w, double dt);
};
#endif