//
// Created by ZYF on 24-6-26.
//
#include "smooth_controller.hpp"

float SmoothController::Update(const std::string &attr_name,int track_id,float score,float safty_score,int smooth_window)
{
  auto it = m_attr_map.find(attr_name);

  if(it == m_attr_map.end())
  {
    AddAttribute(attr_name,safty_score,smooth_window);

    return safty_score;
  }

  return it->second.Update(track_id,score);
}

void SmoothController::AddAttribute(const std::string &attr_name,float safty_score,int smooth_window)
{
  Smoother smoother(safty_score,smooth_window);
  m_attr_map[attr_name] = std::move(smoother);

  return;
}