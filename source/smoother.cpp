//
// Created by ZYF on 24-6-26.
//

#include "smoother.hpp"

float Smoother::Update(int track_id,float score)
{
  auto it = m_attr_map.find(track_id);
  if(it == m_attr_map.end())
  {
    AddTrackId(track_id);

    return m_safty_score;
  }

  return it->second.Update(score);
}

void Smoother::AddTrackId(int track_id)
{
  KalmanController kalman_controler(m_smooth_window,m_safty_score);
  m_attr_map[track_id] = std::move(kalman_controler);

  return;
}

