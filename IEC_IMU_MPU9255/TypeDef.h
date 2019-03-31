#ifndef _TYPEDEF_H
#define _TYPEDEF_H

#define T (2500.0/1000000.0)  // sampling period in sec
#define pi 3.1415926536
#define alpha_acc_LPF  (1.0/(1.0+2*pi*10*T))
#define alpha_mag_LPF  (1.0/(1.0+2*pi*10*T))
#define alpha_gyro_HPF  (1.0/(1.0+2*pi*0.02*T))
#define alpha_vel_HPF  (1.0/(1.0+2*pi*0.1*T))
#define mu 9.8
#define mu_acc 0.05
#define mu_mag 0.2
#define sigma_acc 0.4
#define sigma_mag 1.0
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))


typedef struct
{
  int Count;
  
  /* IMU Input */
  double Acc[3];
  double Gyro[3];
  double Mag[3];
  double optical_flow[2];
  double DCM[3][3], dcm[3][3];
  double Acc_M, Acc_M_initial;
  double Mag_M, Mag_M_initial;
  double TP;

  /* Position and Velocity Estimation */
  double Px, Py, Pz, Vx, Vy, Vz, vx, vy, vz;
  double Anx, Any, Anz, anx, any, anz;

  /* LPF for acc and mag */
  double acc_pre[3], mag_pre[3];
  /* HPF for gyro */
  double Gyro_fused[3], Gyro_in_pre[3];
  /* HPF for velocity estimation */
  double V_integration[3], V_sensor[3], V_sensor_pre[3];
  double P_integration[3];
  double Yaw_ini, Pitch_ini, Roll_ini;
  
  /* Quaternion */
  double q[4];
  double q_dcm[4];
  double qa[4];
  double qm[4];
  double qref[4];
  
  // Euler Angles
  double Roll;
  double Pitch;
  double Yaw;

  // Euler Angle Rate
  double Roll_rate;
  double Pitch_rate;
  double Yaw_rate;

  // Complementary filter weighting
  double Wei;

} IMU_VAR;


typedef struct
{
  double T_acc[3][3];
  double b_acc[3];
  double T_mag[3][3];
  double b_mag[3];
  double b_gyro[3];
}IMU_BIAS;


#endif


