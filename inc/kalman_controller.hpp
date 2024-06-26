//
// Created by ZYF on 24-6-26.
//

#ifndef KALMAN_CONTROLER_HPP
#define KALMAN_CONTROLER_HPP


#include "kalman.hpp"
#include <memory>

class KalmanController
{
public:
  KalmanController()
    :m_smooth_window(10),m_safety_score(0.5),m_filter_counter(0),m_y(1)
  {

  }

  KalmanController(int smooth_window,float safety_score);

  float Update(float score);

private:
  int m_smooth_window;
  float m_safety_score;

  int m_filter_counter;

  Eigen::VectorXd m_y;

  std::unique_ptr<KalmanFilter> m_kalman_filter;
};


#endif //KALMAN_CONTROLER_HPP