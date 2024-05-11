﻿#include "ROAR_UKF.h"
#include <iostream>
using namespace std;
ROVER::ROVER()
{
    /***
    ROVER Default Constructor
    ***/
    // Default Constructor with default parameters
    Kinematic_model_parameters = Eigen::MatrixXd::Zero(2, 6);
    rover_speeds = Eigen::VectorXd::Zero(2);
    
    Kinematic_model_parameters << 0.04, 0.04, 0.04, 0.04, 0.04, 0.04,
                                 -0.097, -0.097, -0.097, 0.097, 0.097, 0.097;

}
void ROVER::calculate_wheel_change(Eigen::VectorXd w, double dt)
{
    /****
      Process wheel speeds to calculate rover linear and angular velocity, position and orientation
        Inputs:
        x: current sigma point of state estimate x = [q0 q1 q2 q3 omega_x omega_y omega_z x y].T
        w: Wheel speeds (w1, w2).T
        dt: delta time

        Outputs:
        velocity: Linear velocity of rover
        omega: Angular velocity of rover
    ***/
    rover_speeds = Kinematic_model_parameters * w;
}
ROVER::~ROVER()
{
    // Destructor
}
/*** ------ Sigma points --------- ***/
MerwedSigmaPoints::MerwedSigmaPoints()
{
    /***
    Merwe Sigma Point Default Constructor
    ***/    
    // Default Constructor with default parameters
    this->n = n;
    this->num_sigma_points = int(2*n + 1);

    // Default sigma points params
    this->alpha = 3.0;
    this->beta = 2.0;
    this->kappa = 0.1;

    // Compute the weight matrices
    this->Wm = compute_Wm();
    this->Wc = compute_Wc();
}
MerwedSigmaPoints::MerwedSigmaPoints(int n, double alpha, double beta, double kappa)
{
    /***
    Merwe Sigma Point Generalized Constructor

    Inputs:
    n: state dimension of state estimate
    alpha: alpha param for Merwe Sigma point
    beta: beta param of Merwe Sigma Point
    kappa: kappa param of Merwe Sigma Point
    ***/
    // Num sigma points
    this->n = n;
    this->num_sigma_points = int(2 * n + 1);

    // Default sigma points params
    this->alpha = alpha;
    this->beta = beta;
    this->kappa = kappa;

    // Compute the weight matrices
    this->Wm = compute_Wm();
    this->Wc = compute_Wc();
}
MerwedSigmaPoints::~MerwedSigmaPoints()
{
    // Destructor
}
/*** Weight computation ***/
Eigen::VectorXd MerwedSigmaPoints::compute_Wm()
{
    /***
    Calculates The Weighted Mean for Merwe Sigma Points

    Outputs:
    Wm: (2n+1)x(n) Weight Mean
    ***/

    // Compute lambda
    double lambda_ = alpha * alpha * (n + kappa) - n;

    // Initialize Wm weight array 
    // BLA::Matrix<2*n + 1> Wm;
    Wm = Eigen::VectorXd(num_sigma_points);

    // Compute initial weight
    Wm(0) = lambda_ / (n + lambda_);

    // Compute the rest of the weight
    for (int i = 1; i < 2 * n + 1; i++)
    {
        Wm(i) = 1.0 / (2 * (n + lambda_));
    }

    return Wm;
}
Eigen::VectorXd MerwedSigmaPoints::compute_Wc()
{
    /***
    Calculates The Weighted Covariance for Merwe Sigma Points

    Outputs:
    Wc: (2n+1)x(n) Weight Covariance
    ***/

    // Compute lambda
    double lambda_ = alpha * alpha * (n + kappa) - n;

    // Initialize Wm weight array 
    // BLA::Matrix<2*n + 1> Wc;
    Wc = Eigen::VectorXd(num_sigma_points);

    // Compute initial weight
    Wc(0) = (lambda_ / (n + lambda_)) + 1 - alpha * alpha + beta;

    // Compute the rest of the weight
    for (int i = 1; i < 2 * n + 1; i++)
    {
        Wc(i) = 1.0 / (2 * (n + lambda_));
    }

    return Wc;

}
/*** Sigma point calculation ***/
Eigen::MatrixXd MerwedSigmaPoints::calculate_sigma_points(Eigen::VectorXd mean, Eigen::MatrixXd cov)
{
    /***
    Calculates Merwe Sigma Points
    Inputs:
    mean "X": nx1 matrix of state mean
    cov "P": nxn covariance matrix

    Outputs:
    sigma_points: (2n+1) Merwe Sigma Points
    ***/
    // Init sigma point array
    Eigen::MatrixXd sigma_points = Eigen::MatrixXd::Zero(n, num_sigma_points); // Sigma points Matrix is not transposed

    // Square root of (n + lambda) * cov
    double lambda_ = alpha * alpha * (n + kappa) - n;
    Eigen::MatrixXd n_plus_lambda_times_cov = (n + lambda_) * cov;

    Eigen::LLT<Eigen::MatrixXd> lltOfA(n_plus_lambda_times_cov);    // compute the Cholesky decomposition of A
    Eigen::MatrixXd U = lltOfA.matrixU();                           // retrieve factor U  in the decomposition (upper)

    // Calculate sigma points
    sigma_points.col(0) = mean; // First column corresponds to the mean
    for (int i = 0; i < n; i++)
    {
        sigma_points.col(i + 1) = mean + U.col(i); // Positive sigma points
        sigma_points.col(i + n + 1) = mean - U.col(i); // Negative sigma points
    }
    return sigma_points;
}
UKF::UKF(MerwedSigmaPoints merwed_sigma_points)
{
    /***
    Unscented Kalman Filter Constructor with Merwe Sigma Points

    This is a default constructor for the Unscented Kalman Filter for Orientation Estimation
    using Quaternions. The constructor takes in Merwe sigma points and assigns it to the UKF.

    The state space is:

    x = [q0 q1 q2 q3 omega_x, omega_y, omega_z x y].T

    where (q0,q1,q2,q3) are the quaternion elements of the following quaternion
    q0 + q1*i + q2*j + q3*k. (omega_x, omega_y, omega_z) is the angular velocity. (x,y) 
    is the position

    The z measurement state space is:

    z = [z_gyro, z_acc, z_mag, z_gps].T

    where z_gyro is the measurement from the gyro, z_acc is the measurement from the accelerometer
    z_mag is the measurement from the magnetometer. Note that these measurements are in the
    body frame. z_gps is the measurement from the GPS (latitude, longitude).
    ***/
    // Initialize x state vector
    x_dim = 9;
    x_hat = Eigen::VectorXd::Zero(x_dim);
    x_hat << 1, 0, 0, 0, 0, 0, 0, 0, 0;   // Initial Quaterion: 1+0i+0j+0k and initial ang vel: [0 0 0].T

    // Initialize z state vector
    z_dim = 11;
    z = Eigen::VectorXd::Zero(z_dim); // Initial measurement in frame {B}, [z_gyro, z_acc, z_mag].T

    // Intialize Posteriori Estimate Covariance Matrix
    P = Eigen::MatrixXd::Zero(x_dim, x_dim);
    S = Eigen::MatrixXd::Zero(z_dim, z_dim);

    // Initialize Prior Estimates
    x_prior = x_hat;
    P_prior = P;

    // Initial Posterior Estimates
    x_post = x_hat;
    P_post = P;

    // Compute mean and covariance using unscented transform
    z_prior = z;
    S_prior = S;

    // Assign the sigma points into UKF class
    sigma_points = merwed_sigma_points;

    X_sigma = Eigen::MatrixXd::Zero(x_dim, sigma_points.num_sigma_points); // Predicted sigma points
    Z_sigma = Eigen::MatrixXd::Zero(z_dim, sigma_points.num_sigma_points); // Measurement sigma points

    // Initialize noise matrices
    Q = Eigen::MatrixXd::Identity(x_dim, x_dim) * 1e-10;    // Process Noise Matrix //research

    R = Eigen::MatrixXd::Identity(z_dim, z_dim) * 0.7;      // Measurement Noise Matrix //add noise covariance for each sensor from datasheet

    // Intialize inertial frame quantities
    g0 << 0, 0, 1;                          // Gravitational Acceleration Vector
    m0 << B_INTENSITY * cos(INCLINATION),   // Magnetic Field Intensity Vector
        0.0,
        B_INTENSITY* sin(INCLINATION);

    yaw = 0.0;

}
/*** Destructor ***/
UKF::~UKF()
{
    // Destructor
}
// --- Unscented Transform ---
std::tuple<Eigen::VectorXd, Eigen::MatrixXd> UKF::unscented_transform(Eigen::MatrixXd sigmas,
    Eigen::MatrixXd Wm,
    Eigen::MatrixXd Wc,
    Eigen::MatrixXd noise_cov)
{
    /***
    Computes the unscented transform from the sigma points, weighted means and weighted noise covariance.

    Inputs:
    sigmas: Sigma points of UKF
    Wm: Weighted Mean
    Wc: Weighted Covariance
    noise_cov: Noise covariance matrix

    Outputs:
    mu: Mean from unscented transform computation
    P_cov: Covariance from unscented transform computation
    ***/
    // Compute new mean
    Eigen::VectorXd mu = sigmas * Wm;   // Vectorization of sum(wm_i* sigma_i)

    // Compute new covariance matrix
    int kmax = sigmas.cols(); // Number of columns instead of rows
    int n = sigmas.rows();    // Number of rows
    Eigen::MatrixXd P_cov = Eigen::MatrixXd::Zero(n, n);

    for (int k = 0; k < kmax; k++)
    {
        Eigen::VectorXd y = sigmas.col(k) - mu;
        P_cov += Wc(k) * y * y.transpose();
    }

    // Add noise
    P_cov += noise_cov;

    return std::make_tuple(mu, P_cov);
}
void UKF::predict_states(Eigen::VectorXd w, double dt) // what is dt
{
    /***
    Predict with wheel odometry process model
    u_t: Measured wheels velocity as input
    ***/
    // Compute the sigma points for given mean and posteriori covariance
    Eigen::MatrixXd sigmas = sigma_points.calculate_sigma_points(x_hat, P);

    // Pass sigmas into f(x) for wheel odometry
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        // Predict with wheel odometry process model for sigma points
        X_sigma.col(i) = process_model(sigmas.col(i), w, dt);
    }

    // Compute unscented mean and covariance
    std::tie(x_hat, P) = unscented_transform(X_sigma,
        sigma_points.Wm,
        sigma_points.Wc,
        Q);

    // Save prior
    x_prior = x_hat.replicate(1, 1);
    P_prior = P.replicate(1, 1);

}
// --- Process & Measurement Model with Wheel Odometry ---
// Process Model
Eigen::VectorXd UKF::process_model(Eigen::VectorXd x, Eigen::VectorXd w, double dt)
{
    /***
    Nonlinear process model for Wheel Odometry

    Inputs:
    x: current sigma point of state estimate x = [q0 q1 q2 q3 omega_x omega_y omega_z x y].T
    u_t: Current input to nonlinear process model
    dt: delta time

    ***/

    // Process wheel speeds using Kinematic Model
    ROVER rover;
    rover.calculate_wheel_change(w, dt);

    // considern changing this quaternion into UnitQuaternion
    Quaternion attitude(x(0),
        x(1),
        x(2),
        x(3));

    // Estimated attitude update with incremental rotation update
    // EQN 3.26 & EQN 3.17 (Exponential with skew matrix and delta_t)
    //consider adding noise to the angular velocity and orientation
    UnitQuaternion uq_omega = UnitQuaternion::omega(x(4) * dt,
        x(5) * dt,
        x(6) * dt);
    attitude = attitude * uq_omega;

    Eigen::VectorXd x_pred_sigma(9);

    // Quaternions
    x_pred_sigma(0) = attitude.s;
    x_pred_sigma(1) = attitude.v_1;
    x_pred_sigma(2) = attitude.v_2;
    x_pred_sigma(3) = attitude.v_3;

    // Angular velocity
    x_pred_sigma(4) = x(4);
    x_pred_sigma(5) = x(5);
    x_pred_sigma(6) = x(6);

    //position
    float yaw = atan2(2 * (x(0) * x(3) + x(1) * x(2)), (1 - 2 * (x(2) * x(2) + x(3) * x(3))));
    // Update position based on linear and angular velocities
    // cout << "yaw: " << yaw << endl;

    // Update x and y positions
    double linear_velocity = round(rover.rover_speeds(0)*100)/100;
    double angular_velocity = round(rover.rover_speeds(1)*100)/100;

    // Calculate change in x and y positions
    double dx = linear_velocity * cos(yaw) * dt;
    double dy = linear_velocity * sin(yaw) * dt;

    // Update x and y positions
    x_pred_sigma(7) = x(7) + dx;
    x_pred_sigma(8) = x(8) + dy;
    return x_pred_sigma;
}
void UKF::predict_measurement(double dt, Eigen::VectorXd w, double lon0, double lat0)
{
    /***
    Update step of UKF with Quaternion + Angular Velocity model i.e state space is:

    x = [q0 q1 q2 q3 omega_x omega_y omega_z].T
    z = [z_gyro z_acc z_mag].T

    Inputs:
    z_measurement: Sensor measurements from gyroscope, accelerometer and magnetometer
    ***/
   
    // Pass the transformed sigmas into measurement function
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        // Update sigmas with measurement model
        Z_sigma.col(i) = UKF::measurment_model(X_sigma.col(i) , w, lon0, lat0, dt);
    }

    std::tie(z_prior, S) = unscented_transform(Z_sigma,
        sigma_points.Wm,
        sigma_points.Wc,
        R);
}
// Measurement Model
Eigen::VectorXd UKF::measurment_model(Eigen::VectorXd x, Eigen::VectorXd w, double lon0, double lat0, double dt)
{
    /***
    Nonlinear measurement model for Orientation estimation with Quaternions

    Inputs:
    x: current sigma point of state estimate x = [q0 q1 q2 q3 omega_x omega_y omega_z].T

    Outputs:
    z_pred_sigma: sigma point after being propagated through nonlinear measurement model
    ***/
    // --- Measurement model ---
    // Extract quaternion from current state estimates 
    UnitQuaternion attitude(x(0),
        x(1),
        x(2),
        x(3));

    // Inverse: {B} to {0}
    UnitQuaternion invq = attitude.inverse();

    // Accelerometer
    Eigen::VectorXd acc_pred = invq.vector_rotation_by_quaternion(g0);

    // Magnetomer
    Eigen::VectorXd mag_pred = invq.vector_rotation_by_quaternion(m0);

    // Gyroscope
    Eigen::VectorXd gyro_pred(3);
    gyro_pred << x(4), x(5), x(6);

    // float yaw = atan2(2 * (x(0) * x(3) + x(1) * x(2)), 1 - 2 * (x(2) * x(2) + x(3) * x(3)));
    // yaw += round(rover.rover_speeds(1) * dt*100)/100; // Update orientation


    ROVER rover;
    rover.calculate_wheel_change(w, dt);
    
    // Update position based on linear and angular velocities
    yaw += rover.rover_speeds(1) * dt; // Update orientation

    // Update x and y positions
    double linear_velocity = rover.rover_speeds(0);
    double angular_velocity = rover.rover_speeds(1);

    // Calculate change in x and y positions
    double dx = linear_velocity * cos(yaw) * dt;
    double dy = linear_velocity * sin(yaw) * dt;
    double lat = lat0 + (180 / PI) * (dy / 6378137);
    double lon = lon0 + (180 / PI) * (dx / 6378137) / cos(lat0);

    // Z prediction
    Eigen::VectorXd z_pred_sigma(11);
    z_pred_sigma << gyro_pred, acc_pred, mag_pred, lat, lon;

    return z_pred_sigma;

}
void UKF::update(Eigen::MatrixXd z_measurement)
{
	/***
    	Update step of UKF with Quaternion + Angular Velocity model i.e state space is:
        
        	x = [q0 q1 q2 q3 omega_x omega_y omega_z].T
            	z = [z_gyro z_acc z_mag].T
                
                	Inputs:
                    	z_measurement: Sensor measurements from gyroscope, accelerometer and magnetometer
                        	***/

	// Compute cross covariance
	Eigen::MatrixXd T = Eigen::MatrixXd::Zero(x_dim, z_dim);
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
		T = T + sigma_points.Wc(i) * (X_sigma.col(i) - x_hat) * (Z_sigma.col(i) - z_prior).transpose();
	}

	// Compute Kalman gain
	Eigen::MatrixXd K = T * S.inverse();

	// Update state estimate
	x_hat = x_hat + K * (z_measurement - z_prior);

	// Update covariance
	P = P - K * S * K.transpose();

	// Save posterior
	x_post = x_hat.replicate(1, 1);
	P_post = P.replicate(1, 1); 

}
void UKF::encoder_callback(Eigen::VectorXd w, double dt, double yaw)
{
    /***
    Predict with wheel odometry process model
    u_t: Measured wheels velocity as input
    ***/
    // Compute the sigma points for given mean and posteriori covariance
    Eigen::MatrixXd sigmas = sigma_points.calculate_sigma_points(x_post, P_post);

    // Pass sigmas into f(x) for wheel odometry
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        X_sigma.col(i)(0) = sigmas.col(i)(0);
        X_sigma.col(i)(1) = sigmas.col(i)(1);
        X_sigma.col(i)(2) = sigmas.col(i)(2);
        X_sigma.col(i)(3) = sigmas.col(i)(3);
        X_sigma.col(i)(4) = sigmas.col(i)(4);
        X_sigma.col(i)(5) = sigmas.col(i)(5);
        X_sigma.col(i)(6) = sigmas.col(i)(6);

        // Process wheel speeds using Kinematic Model
        ROVER rover;    
        rover.calculate_wheel_change(w, dt);

        //position
        // Update x and y positions
        double linear_velocity = rover.rover_speeds(0);

        // Calculate change in x and y positions
        double dx = linear_velocity * cos(yaw) * dt;
        double dy = linear_velocity * sin(yaw) * dt;

        // Update x and y positions
        X_sigma.col(i)(7) = sigmas.col(i)(7) + dx;
        X_sigma.col(i)(8) = sigmas.col(i)(8) + dy;
    }

    // Compute unscented mean and covariance
    std::tie(x_hat, P) = unscented_transform(X_sigma,
        sigma_points.Wm,
        sigma_points.Wc,
        Q);

    // // Save prior
    x_post.tail(2) = x_hat.tail(2);
    P_post.col(7) = P.col(7);
    P_post.col(8) = P.col(8);
    P_post.row(7) = P.row(7);
    P_post.row(8) = P.row(8);

    x_prior.tail(2) = x_hat.tail(2);
    P_prior.col(7) = P.col(7);
    P_prior.col(8) = P.col(8);
    P_prior.row(7) = P.row(7);
    P_prior.row(8) = P.row(8);


    // cout << "x_hat: " << endl << x_hat.transpose() << endl;
    // cout << "x_post: " << x_post.transpose() << endl;
    // cout << "P_post: " << endl << P_post << endl;

    // cout << "weight mean: " << endl << sigma_points.Wm << endl;
    // cout << "weight cov: " << endl << sigma_points.Wc << endl;
}
void UKF::imu_callback(Eigen::VectorXd z_measurement, double dt)
{
        /***
    Predict with wheel odometry process model
    u_t: Measured wheels velocity as input
    ***/
    // Compute the sigma points for given mean and posteriori covariance
    Eigen::MatrixXd sigmas = sigma_points.calculate_sigma_points(x_post, P_post);

        // Pass sigmas into f(x) for wheel odometry
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        X_sigma.col(i)(4) = sigmas.col(i)(4);
        X_sigma.col(i)(5) = sigmas.col(i)(5);
        X_sigma.col(i)(6) = sigmas.col(i)(6);
        X_sigma.col(i)(7) = sigmas.col(i)(7);
        X_sigma.col(i)(8) = sigmas.col(i)(8);

        // considern changing this quaternion into UnitQuaternion
        UnitQuaternion attitude(sigmas.col(i)(0),
        sigmas.col(i)(1),
        sigmas.col(i)(2),
        sigmas.col(i)(3));

        // Estimated attitude update with incremental rotation update
        // EQN 3.26 & EQN 3.17 (Exponential with skew matrix and delta_t)
        //consider adding noise to the angular velocity and orientation
        UnitQuaternion uq_omega = UnitQuaternion::omega(sigmas.col(i)(4) * dt,
            sigmas.col(i)(5) * dt,
            sigmas.col(i)(6) * dt);  

        attitude = attitude * uq_omega;

        X_sigma.col(i)(0) = attitude.s;
        X_sigma.col(i)(1) = attitude.v_1;
        X_sigma.col(i)(2) = attitude.v_2;
        X_sigma.col(i)(3) = attitude.v_3;
    }

    // Compute unscented mean and covariance
    std::tie(x_hat, P) = unscented_transform(X_sigma,
        sigma_points.Wm,
        sigma_points.Wc,
        Q);

    // Save posterior
    UnitQuaternion uq(x_hat(0), x_hat(1), x_hat(2), x_hat(3));
    uq.normalize();
    x_prior.head(4) = uq.to_quaternion_vector();
    x_prior(4) = z_measurement(0);
    x_prior(5) = z_measurement(1);
    x_prior(6) = z_measurement(2);
    P_prior.topLeftCorner(7,7) = P.topLeftCorner(7,7);

    // Pass the transformed sigmas into measurement function
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
            /***
        Nonlinear measurement model for Orientation estimation with Quaternions

        Inputs:
        x: current sigma point of state estimate x = [q0 q1 q2 q3 omega_x omega_y omega_z].T

        Outputs:
        z_pred_sigma: sigma point after being propagated through nonlinear measurement model
        ***/
        // --- Measurement model ---
        // Extract quaternion from current state estimates 
        UnitQuaternion attitude(X_sigma.col(i)(0),
            X_sigma.col(i)(1),
            X_sigma.col(i)(2),
            X_sigma.col(i)(3));

        // Inverse: {B} to {0}
        UnitQuaternion invq = attitude.inverse();

        // Accelerometer
        Eigen::VectorXd acc_pred = invq.vector_rotation_by_quaternion(g0);

        // Magnetomer
        Eigen::VectorXd mag_pred = invq.vector_rotation_by_quaternion(m0);

        // Gyroscope
        Eigen::VectorXd gyro_pred(3);
        gyro_pred << X_sigma.col(i)(4), X_sigma.col(i)(5), X_sigma.col(i)(6);

        Z_sigma.col(i) << gyro_pred, acc_pred, mag_pred, Z_sigma.col(i)(9), Z_sigma.col(i)(10); // test behaviour for last two values
    }

    std::tie(z, S) = unscented_transform(Z_sigma,
        sigma_points.Wm,
        sigma_points.Wc,
        R);

    z_prior.head(9) = z.head(9);
    S_prior.topLeftCorner(9,9) = S.topLeftCorner(9,9);

    // cout << "weight mean: " << endl << sigma_points.Wm << endl;
    // cout << "weight cov: " << endl << sigma_points.Wc << endl;

    // cout << "z_prior: " << endl << z_prior.transpose() << endl;
    // cout << "S: " << endl << S << endl;

    	/***
    	Update step of UKF with Quaternion + Angular Velocity model i.e state space is:
        
        	x = [q0 q1 q2 q3 omega_x omega_y omega_z].T
            	z = [z_gyro z_acc z_mag].T
                
                	Inputs:
                    	z_measurement: Sensor measurements from gyroscope, accelerometer and magnetometer
                        	***/

	    // Compute cross covariance
	    Eigen::MatrixXd T = Eigen::MatrixXd::Zero(x_dim, z_dim);
        for (int i = 0; i < sigma_points.num_sigma_points; i++)
        {
	    	T = T + sigma_points.Wc(i) * (X_sigma.col(i) - x_prior) * (Z_sigma.col(i) - z_prior).transpose();
	    }

	    // Compute Kalman gain
	    Eigen::MatrixXd K = T * S.inverse();

	    // Update state estimate
	    x_hat = x_hat + K * (z_measurement - z_prior); // x_hat is defined in constructor for as a temp vector (overwriting x_post)

	    // Update covariance
	    P = P_prior - K * S_prior * K.transpose();

	    // Save posterior
	    x_post.head(4) = x_hat.head(4);
        x_post(4) = z_measurement(0);
        x_post(5) = z_measurement(1);
        x_post(6) = z_measurement(2);
	    P_post.topLeftCorner(7,7) = P.topLeftCorner(7,7);
        
        // cout << "T: " << endl << T << endl;
        // cout << "K: " << endl << K << endl;
        // cout << "x_hat: " << endl << x_hat.transpose() << endl;
        // cout << "x_post: " << x_post.transpose() << endl;
        // cout << "P_post: " << endl << P_post << endl;
    
    
    // float roll = atan2(2*(x_post(0)*x_post(1) + x_post(2)
    //                         *x_post(3)), 1 - 2*(x_post(1)*x_post(1) 
    //                             + x_post(2)*x_post(2)))*180/PI;
	// float pitch = asin(2*(x_post(0)*x_post(2) - x_post(3)*x_post(1)))*180/PI;
	// float yaw = atan2(2*(x_post(0)*x_post(3) + x_post(1)
    //                     *x_post(2)), 1 - 2*(x_post(2)*x_post(2)
    //                         + x_post(3)*x_post(3)))*180/PI;

    // cout << "filter output: " << roll << " " << pitch << " " << yaw << endl;
    // cout << "x_post: " << x_post.transpose() << endl;

    // cout << "orientation: " << x_post(0) << " " << x_post(1) << " " << x_post(2) << " " << x_post(3) << endl;
}

void UKF::gps_callback( Eigen::VectorXd z_measurement, double lon0, double lat0, double yaw)
{
    /***
    Predict with wheel odometry process model
    u_t: Measured wheels velocity as input
    ***/
    // Compute the sigma points for given mean and posteriori covariance
    Eigen::MatrixXd sigmas = sigma_points.calculate_sigma_points(x_post, P_post);

    // Pass sigmas into f(x) for wheel odometry
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        double lat = lat0 + (180 / PI) * (sigmas.col(i)(7) / 6378137);
        double lon = lon0 + (180 / PI) * (sigmas.col(i)(8) / 6378137) / cos(lat0);

        Z_sigma.col(i) << Z_sigma.col(i)(0), Z_sigma.col(i)(1), Z_sigma.col(i)(2), Z_sigma.col(i)(3), Z_sigma.col(i)(4), Z_sigma.col(i)(5), Z_sigma.col(i)(6),
                        Z_sigma.col(i)(7), Z_sigma.col(i)(8), lat, lon;

    }

    std::tie(z, S) = unscented_transform(Z_sigma,
        sigma_points.Wm,
        sigma_points.Wc,
        R);

    z_prior.tail(2) = z.tail(2);
    S_prior.col(9)  = S.col(9);
    S_prior.col(10) = S.col(10);
    S_prior.row(9)  = S.row(9);
    S_prior.row(10) = S.row(10);

    	/***
    	Update step of UKF with Quaternion + Angular Velocity model i.e state space is:
        
        	x = [q0 q1 q2 q3 omega_x omega_y omega_z].T
            	z = [z_gyro z_acc z_mag].T
                
                	Inputs:
                    	z_measurement: Sensor measurements from gyroscope, accelerometer and magnetometer
                        	***/

    // Compute cross covariance
    Eigen::MatrixXd T = Eigen::MatrixXd::Zero(x_dim, z_dim);
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        T = T + sigma_points.Wc(i) * (X_sigma.col(i) - x_prior) * (Z_sigma.col(i) - z_prior).transpose();
    }

    // Compute Kalman gain
    Eigen::MatrixXd K = T * S.inverse();

    // Update state estimate
    x_hat = x_hat + K * (z_measurement - z_prior); // x_hat is defined in constructor for as a temp vector (overwriting x_post)

    // Update covariance
    P = P_prior - K * S_prior * K.transpose();

    // Save posterior
    x_post.tail(2) = x_hat.tail(2);
    P_post.col(7) = P.col(7);
    P_post.col(8) = P.col(8);
    P_post.row(7) = P.row(7);
    P_post.row(8) = P.row(8);
}