#ifndef __POSE_EKF_H
#define __POSE_EKF_H
#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
#define GRAVITY_MSS 9.80665f

Quaterniond euler2quaternion(Vector3d euler);
Matrix3d quaternion2mat(Quaterniond q);
Vector3d mat2euler(Matrix3d m);
Quaterniond mat2quaternion(Matrix3d m);
Matrix3d euler2mat(Vector3d euler);
Vector3d quaternion2euler(Quaterniond q);
// state for kalman filter
// 0-2 Px Py Pz
// 3-5 Vx Vy Vz
// 6-8 bax bay baz
// inertial frame: NWU

class Pose_ekf
{
public:
    Pose_ekf();
	~Pose_ekf();	
    void predict(Vector4d q, Vector3d acc, double t);
	void correct(Vector3d pos, Vector3d vel, Vector3d mag, double t);
    void process(Vector4d _quan, Vector3d acc, VectorXd& xdot, MatrixXd& F, MatrixXd& G);
	MatrixXd computeF(Vector3d gyro, Vector3d acc);
	
	VectorXd measurement(VectorXd x, Vector3d mag);
	MatrixXd computeH(Vector3d mag);

	void measurement_fix(Vector2d& position, MatrixXd &H);
	void measurement_fix_velocity(Vector3d& velocity, MatrixXd& H);
	void measurement_sonar_height(VectorXd& sonar_height, MatrixXd& H);
    void measurement_optcalflow(Vector2d& velocity, MatrixXd &H);
    void measurement_posecorrect(Vector2d& dpos, MatrixXd &H);
    void measurement_slam_pose(Vector3d& position, MatrixXd &H);
    void measurement_vicon_pose(Vector3d& position, MatrixXd &H);
    void measurement_vicon_vel(Vector3d& velocity, MatrixXd &H);
	void measurement_gravity(Vector3d& acc, MatrixXd& H);

	void correct(VectorXd z, VectorXd zhat, MatrixXd H, MatrixXd R);
	void correct_fix(Vector3d position, double t);
	void correct_fix_velocity(Vector3d velocity, double t);
	void correct_sonar_height(double sonar_height, double t);//todo, without considering the roll and pitch
    void correct_opt_velocity(Vector2d opt_velocity, double t);
    void correct_posecorrect(Vector2d d_position,Vector4d d_q,double t);
    void correct_slam_pose(Vector3d position, double t);
    void correct_vicon_pose(Vector3d position, double t);
    void correct_vicon_velocity(Vector3d velocity, double t);
	void correct_gravity(Vector3d acc, double t);
	// void measurement_altimeter(double& altimeter_height, MatrixXd H);
    void getState(Quaterniond& q, Vector3d& position, Vector3d& velocity, Vector3d&  ba);
	double get_time() { return current_t;}
    void getposecorrectCov(Vector2d d_position);

    double opt_q;
    Vector3d px4PosInit;
    bool px4pose_initialized;
private:
	VectorXd x;//state 
	MatrixXd P;//covariance


    const Vector3d GRAVITY = Vector3d(0, 0, GRAVITY_MSS);
	//covariance parameter
	const double fix_cov = 2.0;
    const double sonar_height_cov = 0.2;
	const double fix_velocity_cov = 2.0;
	
    const double acc_cov = 0.1;

    const double gravity_cov = 5.0;

    const double optical_cov=0.01;

    const double posecorrect_cov=0.1;

    const double slam_cov=0.001;

    const double vicon_pose_cov=1e-10;

    const double vicon_vel_cov=1e-8;

    const int n_state = 9;
	MatrixXd Q;//imu observation noise
	const MatrixXd R_fix = Matrix2d::Identity()*fix_cov;
	const MatrixXd R_fix_velocity = Matrix3d::Identity()*fix_velocity_cov;
	const MatrixXd R_sonar_height = MatrixXd::Identity(1, 1)*sonar_height_cov;
	const MatrixXd R_gravity = Matrix3d::Identity()*gravity_cov;
    const MatrixXd R_optcal_velocity = Matrix2d::Identity()*optical_cov;
    const MatrixXd R_posecorrect = Matrix2d::Identity()*posecorrect_cov;
    const MatrixXd R_slam_pose= Matrix3d::Identity()*slam_cov;
    const MatrixXd R_vicon_pose= Matrix3d::Identity()*vicon_pose_cov;
    const MatrixXd R_vicon_vel= Matrix3d::Identity()*vicon_vel_cov;

	Vector3d acc;
    Vector4d quan;

	double current_t;
	bool initialized;

	bool fix_initialized;
	bool imu_initialized;
	bool altimeter_initialized;
	bool sonar_initialized;
    bool optical_initialized;
	
};


#endif
