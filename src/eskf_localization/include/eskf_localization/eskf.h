#ifndef __ESKF__
#define __ESKF__

#include <iostream>
#include "math.h"
#include "variable.h"

using namespace std;

class ESKF
{
    private:
        //data init
        double position_noise = 100; //1.2;
        double velocity_noise = 10; //10.0;
        double posture_noise  = 1; //1.0;
        
        //Predict 
        double acc_noise = 5e-4;
        double gyro_noise = 5e-4;
        double acc_bias_noise = 5e-4;
        double gyro_bias_noise = 5e-4;

        Eigen::Matrix <double,18,18> Fx;
        Eigen::Matrix <double,18,12> Fi;
        Eigen::Matrix <double,12,12> Qi;

        // New 
        // Eigen::Matrix <double,9,9> Fx;
        // Eigen::Matrix <double,9,6> Fi;
        // Eigen::Matrix <double,6,6> Qi;

        //Update
        double pose_noise = 1.2; //1.2;
        Eigen::Matrix <double,3,18> H;

        // New
        // Eigen::Matrix <double,3,9> H;

    public:
        // Init
        ESKF();
        ~ESKF();

        // ESKF state estimation 
        void Init(const GPS_Data& gps_data, State& state);
        void Predict(const IMU_Data& imu_data, State& state);
        void Correct(const GPS_Data& gps_data, State& state); 
        void State_update(State& state);
        void Error_State_Reset(State& state);


        // Predict
        //Eigen::Matrix <double,18,18> Jacobian_Fx(Eigen::Vector3d acc,Eigen::Vector3d acc_bias, Eigen::Matrix3d R , const double dt);
        Eigen::Matrix <double,18,18> Jacobian_Fx(Eigen::Vector3d acc,Eigen::Vector3d acc_bias, Eigen::Vector3d gyro, Eigen::Vector3d gyro_bias, Eigen::Matrix3d R, const double dt);
        Eigen::Matrix <double,18,12> Jacobian_Fi();
        Eigen::Matrix <double,12,12> Noise_Qi(const double dt);

        // Eigen::Matrix <double,9,9> Jacobian_Fx(Eigen::Vector3d acc,Eigen::Vector3d acc_bias, Eigen::Vector3d gyro, Eigen::Vector3d gyro_bias, Eigen::Matrix3d R, const double dt);
        // Eigen::Matrix <double,9,6> Jacobian_Fi();
        // Eigen::Matrix <double,6,6> Noise_Qi(const double dt);

        // Correct 
        Eigen::Matrix <double,3,18> Jacobian_H(State& state);

        // Eigen::Matrix <double,3,9> Jacobian_H(State& state);

        // Quaternion 
        Eigen::Quaterniond QuaternionFromAngle(Eigen::Vector3d vec);
        Eigen::Matrix3d skewsym_matrix(const Eigen::Vector3d& vec);
        Eigen::Quaterniond QuaternionFromEuler(Eigen::Vector3d euler);
        Eigen::Quaterniond kronecker_product(const Eigen::Quaterniond& p, const Eigen::Quaterniond& q);
        Eigen::Matrix <double,3,3> RotationFromAngle(Eigen::Vector3d vec);

};

ESKF::ESKF()
{
    cout << "ESKF Start Duy" << endl;
}

ESKF::~ESKF()
{
    cout << "ESKF Finish Duy" << endl;
}

// INIT
void ESKF::Init(const GPS_Data& gps_data, State& state)  //(khoi tao X,P)
{
    state.timestemp = gps_data.timestemp;
    state.position = gps_data.ned;

    state.PEst.block<3, 3>(0, 0) = position_noise * Eigen::Matrix3d::Identity();
    state.PEst.block<3, 3>(3, 3) = velocity_noise * Eigen::Matrix3d::Identity();
    state.PEst.block<3, 3>(6, 6) = posture_noise * Eigen::Matrix3d::Identity();
}

/////////////////////// PREDICT //////////////////////////////////
void ESKF::Predict(const IMU_Data& imu_data, State& state)
{
    const double denta_t = imu_data.timestemp - state.timestemp;
    const double denta_2t = denta_t * denta_t;
    state.timestemp = imu_data.timestemp;

    Eigen::Matrix3d R = state.quaternion.toRotationMatrix();
    Eigen::Vector3d q_v = (imu_data.gyro - state.gyro_bias)*denta_t;

    // printf


    //Fx = Jacobian_Fx(imu_data.acc, state.acc_bias, R, denta_t);
    Fx = Jacobian_Fx(imu_data.acc, state.acc_bias, imu_data.gyro, state.gyro_bias, R, denta_t);

    Fi = Jacobian_Fi();

    Qi = Noise_Qi(denta_t);
    
    // IMU process
    state.position = state.position + denta_t*state.velocity + 0.5 * denta_2t*(R * (imu_data.acc - state.acc_bias) + state.gravity);
    state.velocity = state.velocity + 0.5 * denta_2t*(R * (imu_data.acc - state.acc_bias) + state.gravity);
    state.quaternion = kronecker_product(state.quaternion, QuaternionFromEuler((imu_data.gyro - state.gyro_bias)*denta_t));
    // state.quaternion = state.quaternion * QuaternionFromAngle(q_v);

    state.PEst = Fx * state.PEst * Fx.transpose() + Fi * Qi * Fi.transpose();

    // Print the matrix in COO format
    // std::cout << "Fx" << std::endl;
    // std::cout << Fx << std::endl;
    // std::cout << "Quaternion (w, x, y, z): " << state.quaternion.w() << ", " << state.quaternion.x() << ", " << state.quaternion.y() << ", " << state.quaternion.z() << std::endl;
    // 

}

//JACOBIAN_FX 
// Eigen::Matrix <double, 18,18> ESKF::Jacobian_Fx(Eigen::Vector3d acc,Eigen::Vector3d acc_bias, Eigen::Matrix3d R , const double dt)
// {
//     Eigen::Matrix <double,18,18> Fx = Eigen::Matrix <double,18,18>::Identity();
//     Fx.block<3,3>(0,3) = Eigen::Matrix3d::Identity()*dt;
//     Fx.block<3,3>(3,6) = - skewsym_matrix(R * (acc - acc_bias))*dt;
//     Fx.block<3, 3>(3, 9) = - R*dt;
//     Fx.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity()*dt;
//     Fx.block<3, 3>(6, 12) = - R*dt;
//     return Fx;
// }

//JACOBIAN_FX_2
Eigen::Matrix <double, 18,18> ESKF::Jacobian_Fx(Eigen::Vector3d acc,Eigen::Vector3d acc_bias, Eigen::Vector3d gyro, Eigen::Vector3d gyro_bias, Eigen::Matrix3d R, const double dt)
{
    Eigen::Matrix <double,18,18> Fx = Eigen::Matrix <double,18,18>::Identity();
    Fx.block<3,3>(0,3) = Eigen::Matrix3d::Identity()*dt;
    Fx.block<3,3>(3,6) = - R*skewsym_matrix((acc - acc_bias))*dt;
    Fx.block<3, 3>(3,9) = - R*dt;
    Fx.block<3, 3>(6,6) = RotationFromAngle((gyro - gyro_bias) * dt);
    Fx.block<3, 3>(3,15) = Eigen::Matrix3d::Identity()*dt;
    Fx.block<3, 3>(6,12) = - R*dt;
    return Fx;
}

// Eigen::Matrix <double, 9,9> ESKF::Jacobian_Fx(Eigen::Vector3d acc,Eigen::Vector3d acc_bias, Eigen::Vector3d gyro, Eigen::Vector3d gyro_bias, Eigen::Matrix3d R, const double dt)
// {
//     Eigen::Matrix <double,9,9> Fx = Eigen::Matrix <double,9,9>::Identity();
//     Fx.block<3,3>(0,3) = Eigen::Matrix3d::Identity()*dt;
//     Fx.block<3,3>(3,6) = - R*skewsym_matrix((acc - acc_bias))*dt;
//     Fx.block<3,3>(6,6) = RotationFromAngle((gyro - gyro_bias) * dt);  // or bo dong nay
//     return Fx;
// }

//JACOBIAN_FI
Eigen::Matrix <double,18,12> ESKF::Jacobian_Fi()
{
    Eigen::Matrix <double,18,12> Fi = Eigen::Matrix <double,18,12>::Zero();
    Fx.block<12,12>(3,0) = Eigen::Matrix<double, 12, 12>::Identity();
    return Fi;
}

// Eigen::Matrix <double,9,6> ESKF::Jacobian_Fi()
// {
//     Eigen::Matrix <double,9,6> Fi = Eigen::Matrix <double,9,6>::Zero();
//     Fx.block<6,6>(3,0) = Eigen::Matrix<double, 6, 6>::Identity();
//     return Fi;
// }

//NOISE_QI
Eigen::Matrix <double,12,12> ESKF::Noise_Qi(const double dt)
{
    Eigen::Matrix <double,12,12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    Qi.block<3, 3>(0, 0) = dt * dt * acc_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(3, 3) = dt * dt * gyro_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(6, 6) = dt * acc_bias_noise * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(9, 9) = dt * gyro_bias_noise * Eigen::Matrix3d::Identity();
    return Qi;
}

// Eigen::Matrix <double,6,6> ESKF::Noise_Qi(const double dt)
// {
//     Eigen::Matrix <double,6,6> Qi = Eigen::Matrix<double, 6, 6>::Zero();
//     Qi.block<3, 3>(0, 0) = dt * dt * acc_noise * Eigen::Matrix3d::Identity();
//     Qi.block<3, 3>(3, 3) = dt * dt * gyro_noise * Eigen::Matrix3d::Identity();
//     return Qi;
// }

/////////////////////// CORRECT //////////////////////////////////
void ESKF::Correct(const GPS_Data& gps_data, State& state)
{
    Eigen::Vector3d Y(gps_data.ned[0], gps_data.ned[1], gps_data.ned[2]);
    Eigen::Vector3d X(state.position[0], state.position[1], state.position[2]);
    Eigen::Matrix3d R = pose_noise * Eigen::Matrix3d::Identity();

    H = Jacobian_H(state);

    Eigen::MatrixXd K = state.PEst * H.transpose() * (H * state.PEst * H.transpose() + R).inverse();
    state.error = K * (Y - X);

    state.PEst = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * state.PEst;
    //state.PEst = (Eigen::Matrix<double, 9, 9>::Identity() - K * H) * state.PEst;
}

// Jacobian H
Eigen::Matrix <double,3,18> ESKF::Jacobian_H(State& state)
{
    Eigen::Matrix <double,3,19> Hx = Eigen::Matrix <double,3,19> ::Zero();
    Hx.block<3,3> (0,0) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double,19,18> Xx = Eigen::Matrix<double,19,18>::Identity();
    Eigen::Matrix<double, 4, 3> Q_theta;
    Q_theta << -state.quaternion.x(), -state.quaternion.y(), -state.quaternion.z(),
               state.quaternion.w(), -state.quaternion.z(), state.quaternion.y(),
               state.quaternion.z(), -state.quaternion.w(), -state.quaternion.x(),
               -state.quaternion.y(), state.quaternion.x(), state.quaternion.w();
    Q_theta = Q_theta*0.5;
    Xx.block<4,3>(6,6) = Q_theta;

    Eigen::Matrix <double,3,18> H = Hx*Xx;

    return H;
}

// New
// Eigen::Matrix <double,3,9> ESKF::Jacobian_H(State& state)
// {
//     Eigen::Matrix <double,3,10> Hx = Eigen::Matrix <double,3,10> ::Zero();
//     Hx.block<3,3> (0,0) = Eigen::Matrix3d::Identity();

//     Eigen::Matrix<double,10,9> Xx = Eigen::Matrix<double,10,9>::Identity();
//     Eigen::Matrix<double, 4, 3> Q_theta;
//     Q_theta << -state.quaternion.x(), -state.quaternion.y(), -state.quaternion.z(),
//                state.quaternion.w(), -state.quaternion.z(), state.quaternion.y(),
//                state.quaternion.z(), -state.quaternion.w(), -state.quaternion.x(),
//                -state.quaternion.y(), state.quaternion.x(), state.quaternion.w();
//     Q_theta = Q_theta*0.5;
//     Xx.block<4,3>(6,6) = Q_theta;

//     Eigen::Matrix <double,3,9> H = Hx*Xx;

//     return H;
// }

// Eigen::Matrix <double,3,9> ESKF::Jacobian_H(State& state)
// {
//     Eigen::Matrix <double,3,9> Hx = Eigen::Matrix <double,3,9> ::Zero();
//     Hx.block<3,3> (0,0) = Eigen::Matrix3d::Identity();
//     return Hx;
// }
/////////////////////// State_Update //////////////////////////////////
void ESKF::State_update(State& state)
{
    Eigen::Vector3d error_pos = Eigen::Map<Eigen::Vector3d>(state.error.block<3, 1>(0, 0).transpose().data());
    Eigen::Vector3d error_vel = Eigen::Map<Eigen::Vector3d>(state.error.block<3, 1>(3, 0).transpose().data());
    
    Eigen::Vector3d error_ori = Eigen::Map<Eigen::Vector3d>(state.error.block<3, 1>(6, 0).transpose().data());
    Eigen::Quaterniond error_quat = QuaternionFromEuler(error_ori);

    Eigen::Vector3d error_acc_bias = Eigen::Map<Eigen::Vector3d>(state.error.block<3, 1>(9, 0).transpose().data());
    Eigen::Vector3d error_gyr_bias = Eigen::Map<Eigen::Vector3d>(state.error.block<3, 1>(12, 0).transpose().data());
    Eigen::Vector3d error_gra = Eigen::Map<Eigen::Vector3d>(state.error.block<3, 1>(15, 0).transpose().data());

    state.position = state.position + error_pos;
    state.velocity = state.velocity + error_vel;
    //state.quaternion = state.quaternion*QuaternionFromAngle(error_ori);
    state.quaternion = kronecker_product(error_quat, state.quaternion);
    // state.quaternion = kronecker_product(state.quaternion, error_quat);


    state.acc_bias = state.acc_bias + error_acc_bias;
    state.gyro_bias = state.gyro_bias + error_gyr_bias;
    state.gravity = state.gravity + error_gra;

}
//////////////////// RESET_ERROR /////////////////////////////////
void ESKF::Error_State_Reset(State& state)
{
    state.error.setZero();
}

//////////////////// QUATERNION /////////////////////////////////
Eigen::Quaterniond ESKF::kronecker_product(const Eigen::Quaterniond& p, const Eigen::Quaterniond& q)
{
    Eigen::Quaterniond res;
    res.w() = p.w() * q.w() - p.x() * q.x() - p.y() * q.y() - p.z() * q.z();
    res.x() = p.w() * q.x() + p.x() * q.w() + p.y() * q.z() - p.z() * q.y();
    res.y() = p.w() * q.y() - p.x() * q.z() + p.y() * q.w() + p.z() * q.x();
    res.z() = p.w() * q.z() + p.x() * q.y() - p.y() * q.x() + p.z() * q.w();
    return res;
}

// angle --> quaternion
Eigen::Quaterniond ESKF::QuaternionFromAngle(Eigen::Vector3d vec)
{
    double theta = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
    if (theta == 0)
    {
        Eigen::Vector4d vq;
        vq[0] = 1;
        vq[1] = 0;
        vq[2] = 0;
        vq[3] = 0;
        Eigen::Quaterniond q(vq);
        return q;
    }
    Eigen::Vector3d unit_axis = vec/theta;
    double w = cos(0.5*theta);
    double x = unit_axis[0] * sin(0.5*theta);
    double y = unit_axis[1] * sin(0.5*theta);
    double z = unit_axis[2] * sin(0.5*theta);

    Eigen::Quaterniond qua_from_angle(w,x,y,z);
    qua_from_angle.normalized();
    return qua_from_angle;
}

Eigen::Matrix3d ESKF::skewsym_matrix(const Eigen::Vector3d& vec)
{
    Eigen::Matrix3d mat;
    mat << 0, -vec(2), vec(1),
           vec(2), 0, -vec(0),
          -vec(1), vec(0), 0;
    return mat;
}

Eigen::Quaterniond ESKF::QuaternionFromEuler(Eigen::Vector3d euler)
{
    double roll = euler[0];
    double pitch = euler[1];
    double yaw = euler[2];

    double cr = cos(0.5 * roll);
    double sr = sin(0.5 * roll);
    double cp = cos(0.5 * pitch);
    double sp = sin(0.5 * pitch);
    double cy = cos(0.5 * yaw);
    double sy = sin(0.5 * yaw);

    Eigen::Vector4d vq;
    vq[0] = cy * cp * cr + sy * sp * sr;
    vq[1] = cy * cp * sr - sy * sp * cr;
    vq[2] = sy * cp * sr + cy * sp * cr;
    vq[3] = sy * cp * cr - cy * sp * sr;

    Eigen::Quaterniond qua_from_euler(vq);
    return qua_from_euler;
}

Eigen::Matrix <double,3,3> ESKF::RotationFromAngle(Eigen::Vector3d vec)
{
    double theta = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
    if (0 == theta) {
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Matrix<double, 3, 1> unit_axis = vec / theta;
    Eigen::Matrix<double, 3, 3> unit_mat = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 3> R;
    double sin_th = sin(theta);
    double cos_th = cos(theta);
    R = unit_mat * cos_th + (1 - cos_th) * unit_axis * unit_axis.transpose() + sin_th * skewsym_matrix(unit_axis);

    return R;
}

#endif 