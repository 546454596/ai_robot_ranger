#include "pose_ekf.h"
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


//quaternion: body fram to navigation frame
//Rnb
// state for kalman filter
// 0-2 Px Py Pz
// 3-5 Vx Vy Vz
// 6-8 bax bay baz
// inertial frame: NWU

Matrix3d skew_symmetric(Vector3d v)
{
	Matrix3d m;
	m << 0, -v(2), v(1),
		 v(2), 0,  -v(0),
		 -v(1), v(0), 0;
	return m;
}

//diff_(p*q) /diff_q
Matrix4d diff_pq_q(Quaterniond p)
{
	double p0 = p.w();
	Vector3d pv = p.vec();

	Matrix4d D;
	D(0, 0) = p0;
	D.block<1, 3>(0, 1) = -pv.transpose();
	D.block<3, 1>(1, 0) = pv;
	D.block<3, 3>(1, 1) = Matrix3d::Identity()*p0 + skew_symmetric(pv);
	return D;
}


//diff_(p*q)/ diff_p
Matrix4d diff_pq_p(Quaterniond q)
{
	double q0 = q.w();
	Vector3d qv = q.vec();
	Matrix4d D;
	D(0, 0) = q0;
	D.block<1, 3>(0, 1) = -qv.transpose();
	D.block<3, 1>(1, 0) = qv;
	D.block<3, 3>(1, 1) = Matrix3d::Identity()*q0 - skew_symmetric(qv);
	return D;
}

//diff_(q*v*q_star)/ diff_q
MatrixXd diff_qvqstar_q(Quaterniond q, Vector3d v)
{
	double q0 = q.w();
	Vector3d qv = q.vec();
	MatrixXd D(3, 4);
	D.col(0) = 2*(q0*v + skew_symmetric(qv)*v);
	D.block<3, 3>(0, 1) = 2*(-v*qv.transpose() + v.dot(qv)*Matrix3d::Identity() - q0*skew_symmetric(v));
	return D; 
}

//diff_(qstar*v*q)/ diff_q
MatrixXd diff_qstarvq_q(Quaterniond q, Vector3d v)
{
	double q0 = q.w();
	Vector3d qv = q.vec();
	MatrixXd D(3, 4);
	D.col(0) = 2*(q0*v - skew_symmetric(qv)*v);
	D.block<3, 3>(0, 1) = 2*(-v*qv.transpose() + v.dot(qv)*Matrix3d::Identity() + q0*skew_symmetric(v));
	return D; 
}
//diff_(q*v*q_star)/ diff_v
Matrix3d diff_qvqstar_v(Quaterniond q)
{
	double q0 = q.w();
	Vector3d qv = q.vec();
	Matrix3d D;
	D = (q0*q0 - qv.dot(qv))*Matrix3d::Identity() + 2*qv*qv.transpose() + 2*q0*skew_symmetric(qv);
	return D; 
}

Pose_ekf::Pose_ekf():opt_q(1.0)
{	
	initialized = false;
	x = VectorXd::Zero(n_state);
	P = MatrixXd::Identity(n_state, n_state);

    Q = MatrixXd::Zero(3, 3);
    Q.block<3, 3>(0, 0) = Matrix3d::Identity()*acc_cov;


	initialized = false;
	fix_initialized = false;
	imu_initialized = false;
	altimeter_initialized = false;
	sonar_initialized = false;
    optical_initialized=false;  
}

Pose_ekf::~Pose_ekf()
{
	
}


void Pose_ekf::predict(Vector4d _quan, Vector3d acc, double t)
{
	if(!imu_initialized)
	{
		imu_initialized = true; initialized = true;
		this->current_t = t;
		return;
	}
	if(t <= current_t) return;

	double dt = t - current_t;
	VectorXd xdot(n_state);
	MatrixXd F(n_state, n_state);
    MatrixXd G(n_state, 3);//G = dx/du

    process(_quan, acc, xdot, F, G);
	
    x += xdot*dt;
	F = MatrixXd::Identity(n_state, n_state) + F*dt;//continous F and discrete F
	G = G*dt;
	// cout << "G: " << G << endl;
	// cout << "GQG: " << G*Q*G << endl;

	P = F*P*F.transpose() + G*Q*G.transpose();
	
	this->current_t = t;
	this->acc = acc;
    this->quan = _quan;
}

//xdot = f(x, u);
void Pose_ekf::process(Vector4d _quan, Vector3d acc, VectorXd& xdot, MatrixXd& F, MatrixXd& G)
{
	
	Quaterniond q;
    Vector3d p, v, ba;
    getState(q, p, v, ba);
	
	xdot.setZero();
	F.setZero();
	G.setZero();
	
    xdot.segment<3>(0) = v;

	Quaterniond acc_b_q(0, 0, 0, 0);
    acc_b_q.vec() = acc - ba;
	Quaterniond acc_n_q =  q*acc_b_q*q.inverse();
    xdot.segment<3>(3) = acc_n_q.vec() - GRAVITY;//body frame to n frame


    F.block<3, 3>(0, 3) = Matrix3d::Identity();
    F.block<3, 3>(3, 6) = -diff_qvqstar_v(q);

	//G = d_xdot/du
    G.block<3, 3>(3, 0) = Matrix3d::Identity();//diff_qvqstar_v(q);//diff(q*a*qstar)/diff(a)
}

void Pose_ekf::getState(Quaterniond& q, Vector3d& p, Vector3d& v, Vector3d& ba)
{
    q.w() = this->quan(0);
    q.vec() = this->quan.segment<3>(1);
    p = x.segment<3>(0);
    v = x.segment<3>(3);
    ba = x.segment<3>(6);
}

void Pose_ekf::getposecorrectCov(Vector2d d_position)
{

    R_posecorrect;

}

void Pose_ekf::measurement_fix(Vector2d& position, MatrixXd &H)
{
    position = x.segment<2>(0);
	H = MatrixXd::Zero(2, n_state);
    H.block<2, 2>(0, 0) = Matrix2d::Identity();
}
void Pose_ekf::measurement_fix_velocity(Vector3d& velocity, MatrixXd& H)
{
    velocity = x.segment<3>(3);
	H = MatrixXd::Zero(3, n_state);
    H.block<3, 3>(0, 3) = Matrix3d::Identity();
}

void Pose_ekf::measurement_sonar_height(VectorXd& sonar_height, MatrixXd& H)
{
	sonar_height = VectorXd(1);
    sonar_height(0) = x(2);
	H = MatrixXd::Zero(1, n_state);
    H(0, 2) = 1;
}

void Pose_ekf::measurement_optcalflow(Vector2d& opt_velocity, MatrixXd &H)
{
    opt_velocity = x.segment<2>(3);
    H = MatrixXd::Zero(2, n_state);
    H.block<2, 2>(0, 3) = Matrix2d::Identity();
}

void Pose_ekf::measurement_posecorrect(Vector2d& position, MatrixXd &H)
{
    position = x.segment<2>(0);
    H = MatrixXd::Zero(2, n_state);
    H.block<2, 2>(0, 0) = Matrix2d::Identity();
}

void Pose_ekf::measurement_slam_pose(Vector3d& position, MatrixXd &H)
{
    position = x.segment<3>(0);
    H = MatrixXd::Zero(3, n_state);
    H.block<3, 3>(0, 0) = Matrix3d::Identity();
}

void Pose_ekf::measurement_vicon_pose(Vector3d& position, MatrixXd &H)
{
    position = x.segment<3>(0);
    H = MatrixXd::Zero(3, n_state);
    H.block<3, 3>(0, 0) = Matrix3d::Identity();
}

void Pose_ekf::measurement_vicon_vel(Vector3d& velocity, MatrixXd &H)
{
    velocity = x.segment<3>(3);
    H = MatrixXd::Zero(3, n_state);
    H.block<3, 3>(0, 3) = Matrix3d::Identity();
}


void Pose_ekf::measurement_gravity(Vector3d& acc, MatrixXd& H)
{
	Quaterniond q;
    q.w() = this->quan(0); q.vec() = this->quan.segment<3>(1);
    Vector3d ba = x.segment<3>(6);
	Quaterniond g_n_q;
	g_n_q.w() = 0; g_n_q.vec() = Vector3d(0, 0, 1);//only direction is used
	Quaterniond acc_q =  q.inverse()*g_n_q*q; //r_n to r_b
	acc = acc_q.vec();

	H = MatrixXd::Zero(3, n_state);
    H.block<3, 3>(0, 6) = Matrix3d::Identity();
}


void Pose_ekf::correct(VectorXd z, VectorXd zhat, MatrixXd H, MatrixXd R)
{
   	MatrixXd K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
    x += K*(z - zhat);
    
    MatrixXd I = MatrixXd::Identity(n_state, n_state);
    P = (I - K*H)*P;
}

void Pose_ekf::correct_fix(Vector3d position, double t)
{
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}

	if(t < current_t) return;
	
    predict(this->quan, this->acc, t);
	double dt = t - current_t;
	Vector2d z = position.head(2);
	Vector2d zhat;
	MatrixXd H;
	measurement_fix(zhat, H);
	correct(z, zhat, H, R_fix);
}
void Pose_ekf::correct_fix_velocity(Vector3d velocity, double t)
{
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}

	if(t < current_t) return;
	
    predict(this->quan, this->acc, t);
	
	Vector3d z = velocity;
	Vector3d zhat;
	MatrixXd H;
	measurement_fix_velocity(zhat, H);
	correct(z, zhat, H, R_fix_velocity);
}
void Pose_ekf::correct_sonar_height(double sonar_height, double t)
{
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}

	if(t < current_t) return;	
    predict(this->quan, this->acc, t);

	VectorXd z(1);
	z(0) = sonar_height;
	VectorXd zhat(1);
	MatrixXd H;

    measurement_sonar_height(zhat, H);
    correct(z, zhat, H, R_sonar_height);
}

void Pose_ekf::correct_opt_velocity(Vector2d opt_velocity, double t)
{
    if(!initialized)
    {
        initialized = true;
        this->current_t = t;
        return;
    }

    if(t < current_t) return;

    predict(this->quan, this->acc, t);

    Quaterniond q;
    q.w() = this->quan(0); q.vec() = this->quan.segment<3>(1);

    Matrix3d OptM3= quaternion2mat(q);
    Vector3d velocityBody;
    velocityBody.segment<2>(0)=opt_velocity;
    velocityBody(2)=0;
    Vector3d velocityWorld=OptM3 *velocityBody;
    Vector2d z =velocityWorld.segment<2>(0);

    Vector2d zhat;
    MatrixXd H;
    measurement_optcalflow(zhat, H);
    correct(z, zhat, H, opt_q*R_optcal_velocity);
}

void Pose_ekf::correct_gravity(Vector3d acc, double t)
{
	if(!initialized)
	{
		initialized = true;
		this->current_t = t;
		return;
	}
	if(t < current_t) return;
    predict(this->quan, this->acc, t);
	
	Vector3d z = acc/acc.norm();
	Vector3d zhat;
	MatrixXd H;
	measurement_gravity(zhat, H);
	correct(z, zhat, H, R_gravity);
}

void Pose_ekf::correct_posecorrect(Vector2d d_position,Vector4d d_q,double t)
{
    if(!initialized)
    {
        initialized = true;
        this->current_t = t;
        return;
    }
    if(t < current_t) return;
    Vector2d z = d_position.head(2);
    x.head(2)=z;


}

void Pose_ekf::correct_slam_pose(Vector3d position, double t)
{
    if(!initialized)
    {
        initialized = true;
        this->current_t = t;
        return;
    }

    if(t < current_t) return;

    double dt = t - current_t;
    Vector3d z = position;
    Vector3d zhat;
    MatrixXd H;
    measurement_slam_pose(zhat, H);
    correct(z, zhat, H, R_slam_pose);
}

void Pose_ekf::correct_vicon_pose(Vector3d position, double t)
{
    if(!initialized)
    {
        initialized = true;
        this->current_t = t;
        return;
    }

    if(t < current_t) return;

    double dt = t - current_t;
    Vector3d z = position;
    Vector3d zhat;
    MatrixXd H;
    measurement_vicon_pose(zhat, H);
    correct(z, zhat, H, R_vicon_pose);
}

void Pose_ekf::correct_vicon_velocity(Vector3d velocity, double t)
{
    if(!initialized)
    {
        initialized = true;
        this->current_t = t;
        return;
    }

    if(t < current_t) return;

    double dt = t - current_t;
    Vector3d z = velocity;
    Vector3d zhat;
    MatrixXd H;
    measurement_vicon_vel(zhat, H);
    correct(z, zhat, H, R_vicon_pose);
}

/**
Euler angle defination: zyx
Rotation matrix: C_body2ned
**/
Quaterniond euler2quaternion(Vector3d euler)
{
  double cr = cos(euler(0)/2);
  double sr = sin(euler(0)/2);
  double cp = cos(euler(1)/2);
  double sp = sin(euler(1)/2);
  double cy = cos(euler(2)/2);
  double sy = sin(euler(2)/2);
  Quaterniond q;
  q.w() = cr*cp*cy + sr*sp*sy;
  q.x() = sr*cp*cy - cr*sp*sy;
  q.y() = cr*sp*cy + sr*cp*sy;
  q.z() = cr*cp*sy - sr*sp*cy;
  return q;
}


Matrix3d quaternion2mat(Quaterniond q)
{
  Matrix3d m;
  double a = q.w(), b = q.x(), c = q.y(), d = q.z();
  m << a*a + b*b - c*c - d*d, 2*(b*c - a*d), 2*(b*d+a*c),
       2*(b*c+a*d), a*a - b*b + c*c - d*d, 2*(c*d - a*b),
       2*(b*d - a*c), 2*(c*d+a*b), a*a-b*b - c*c + d*d;
  return m;
}

Vector3d mat2euler(Matrix3d m)
{
  double r = atan2(m(2, 1), m(2, 2));
  double p = asin(-m(2, 0));
  double y = atan2(m(1, 0), m(0, 0));
  Vector3d rpy(r, p, y);
  return rpy;
}

Quaterniond mat2quaternion(Matrix3d m)
{
  //return euler2quaternion(mat2euler(m));
  Quaterniond q;
  double a, b, c, d;
  a = sqrt(1 + m(0, 0) + m(1, 1) + m(2, 2))/2;
  b = (m(2, 1) - m(1, 2))/(4*a);
  c = (m(0, 2) - m(2, 0))/(4*a);
  d = (m(1, 0) - m(0, 1))/(4*a);
  q.w() = a; q.x() = b; q.y() = c; q.z() = d;
  return q;
}

Matrix3d euler2mat(Vector3d euler)
{
  double cr = cos(euler(0));
  double sr = sin(euler(0));
  double cp = cos(euler(1));
  double sp = sin(euler(1));
  double cy = cos(euler(2));
  double sy = sin(euler(2));
  Matrix3d m;
  m << cp*cy,  -cr*sy + sr*sp*cy, sr*sy + cr*sp*cy,
       cp*sy,  cr*cy + sr*sp*sy,  -sr*cy + cr*sp*sy,
       -sp,    sr*cp,             cr*cp;
  return m;
}

Vector3d quaternion2euler(Quaterniond q)
{
  return mat2euler(quaternion2mat(q));
}
