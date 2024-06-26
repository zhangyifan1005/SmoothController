//
// Created by ZYF on 24-6-26.
//

#include "kalman_controller.hpp"

 KalmanController::KalmanController(int smooth_window,float safety_score)
   :m_smooth_window(smooth_window),m_safety_score(safety_score),m_filter_counter(0),m_y(1)
{
   //因为这个平滑只对模型的输出做卡尔曼滤波，所以只需要处理得分本身，所以状态向量与测量向量都可以是一个长度为1的向量
   int n = 1; // Number of states
   int m = 1; // Number of measurements

   double dt = 1.0; // Time step


   Eigen::MatrixXd A(n, n); // System dynamics matrix
   Eigen::MatrixXd C(m, n); // Output matrix
   Eigen::MatrixXd Q(n, n); // Process noise covariance
   Eigen::MatrixXd R(m, m); // Measurement noise covariance
   Eigen::MatrixXd P(n, n); // Estimate error covariance

   // Discrete LTI projectile motion, measuring position only
   A << 1;  //状态转移模型，因为每一次调用都会有一个新得分，所以状态应该在不同的时间保持不变，所以A矩阵应该是值为1的单位矩阵
   C << 1;  //观测模型，由于检测模型直接输出得分，所以直接将状态映射到观测值即可，C矩阵也可以是一个值为1的单位矩阵

   // Reasonable covariance matrices
   Q << 0.001; //过程噪声协方差，这个矩阵描述了状态转移过程中的噪声，让状态变化的不要太快，这个协方差应该设小一点
   R << 0.01;  //测量噪声协方差，这个矩阵描述了测量过程中的噪声，同样认为测量过程中的误差较小，这个协方差应该也设小一点
   P << 1;     //初始估计误差协方差，这里是因初始状态估计不可靠，所以将这个协方差设置为1。

   m_kalman_filter = std::make_unique<KalmanFilter>(dt,A,C,Q,R,P);

   //m_y做初始化，这是一个状态估计值，我这里设置了一个安全值，但是由于初始估计误差协方差矩阵设置的比较大，所以这个值可以说不是很重要。
   //在前10帧只用安全值之后，这个初始值可以说对后续滤波的结果没有影响了。
   m_y << safety_score;
   m_kalman_filter->init(dt,m_y);
}

float KalmanController::Update(float score)
{
   m_y << score;
   m_kalman_filter->update(m_y);

   //前十帧不使用卡尔曼滤波的结果，而使用安全值防止在初始化的时候因为积累的次数不够多导致出错
   if(m_filter_counter < 10)
   {
     m_filter_counter++;

     return m_safety_score;
   }

   return static_cast<float>(m_kalman_filter->state()(0));
}

