//
// Created by huiyan on 10/5/22.
//

#pragma once
#include <vector>
#include <sstream>
#include <ctime>

namespace kitti360 {

template<typename RETURN_TYPE>
struct DateToTimestamp{
  DateToTimestamp()= default;;
  RETURN_TYPE operator()(const std::string &date) const{
    std::stringstream ss(date);
    std::vector<std::string> strs(2);
    std::string temp;
    int cnt = 0;
    while(getline(ss, temp, ' ')) strs[cnt++] = temp;
    if(cnt != 2)  return false; // strs[0] is "2013-05-28" contains year, month, day

    size_t data[5]; // year_, month_, day_, hour_, min_
    RETURN_TYPE second_; // second_
    cnt = 0;
    try{
      ss = std::stringstream(strs[0]);
      while(cnt < 5 && getline(ss, temp, '-')) data[cnt++] = (size_t)std::stod(temp);
      ss = std::stringstream(strs[1]);
      while(cnt < 5 && getline(ss, temp, ':')) data[cnt++] = (size_t)std::stod(temp);
      getline(ss, temp, ':');
      second_ = static_cast<RETURN_TYPE>(stod(temp));
      if(cnt != 5)  return static_cast<RETURN_TYPE>(-1);
    } catch(std::exception &e){
      return false;
    }
    std::tm time_tm{};
    time_tm.tm_year = (int)data[0] - 1900;
    time_tm.tm_mon = (int)data[1] - 1; // index of month from 0
    time_tm.tm_mday = (int)data[2];
    time_tm.tm_hour = (int)data[3];
    time_tm.tm_min = (int)data[4];
    time_tm.tm_sec = 0; // if float number than add in the tail, so it is 0 in there
    time_tm.tm_isdst = -1;
    return static_cast<RETURN_TYPE>(std::mktime(&time_tm)) + second_;
  }
};

}
