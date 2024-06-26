//
// Created by ZYF on 24-6-26.
//

#ifndef SMOOTHER_HPP
#define SMOOTHER_HPP



#include "kalman_controller.hpp"
#include <string>
#include <unordered_map>

class Smoother
{
public:
  Smoother()
    :m_safty_score(0.5),m_smooth_window(10)
  {

  }

  Smoother(float safty_score,int smooth_window)
  :m_safty_score(safty_score),m_smooth_window(smooth_window)
  {

  }

  float Update(int track_id,float score);
private:
  void AddTrackId(int track_id);

  std::unordered_map<int,KalmanController> m_attr_map;

  float m_safty_score;
  int m_smooth_window;
};

#endif //SMOOTHER_HPP
