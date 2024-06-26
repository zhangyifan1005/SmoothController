//
// Created by ZYF on 24-6-26.
//

#ifndef SMOOTH_CONTROLER_HPP
#define SMOOTH_CONTROLER_HPP



#include "smoother.hpp"
#include <string>
#include <unordered_map>

class SmoothController
{
public:
  SmoothController() = default;

  float Update(const std::string &attr_name,int track_id,float score,float safty_score = 0.5f,int smooth_window = 10);
private:
  void AddAttribute(const std::string &attr_name,float safty_score,int smooth_window);

  std::unordered_map<std::string,Smoother> m_attr_map;
};

#endif //SMOOTH_CONTROLER_HPP