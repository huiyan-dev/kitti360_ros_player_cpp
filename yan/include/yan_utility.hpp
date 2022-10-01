#ifndef YAN_UTILITY_HPP
#define YAN_UTILITY_HPP
#include <vector>
#include <sstream>
#include <fstream>
namespace yan{
// ----------------------------------Data Process start----------------------------------------------------------------------
/**
 * @brief for lidar .bin file read / 读取.bin文件的点云,
 * 
 * @tparam T typename
 * @param lidar_data_path 
 * @return std::vector<T>  x, y, z, intensity order to std::vector<T> / 点云按照 x, y, z, intensity 的顺序存进std::vector<T>
 */
template <typename T>
static inline std::vector<T> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(T);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<T> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(T));
    return lidar_data_buffer;
}
template <typename POSE_TYPE, typename RETURN_TYPE>
void read_pose_file(std::ifstream &file, RETURN_TYPE &cache, int r, int c){
  std::string line, s;
  std::getline(file, line);
  std::stringstream ss(line);
  int idx;
  Eigen::Matrix4d p = Eigen::Matrix4d::Identity();
  std::getline(ss, s, ' '); idx = stoi(s) - 1;
  for(int i = 0; i < r; ++i){
    for(int j = 0; j < c; ++j){
      getline(ss, s, ' ');
      p(i, j) = std::stod(s);
    }
  }
  cache = std::make_pair(idx, POSE_TYPE::Identity());
  cache.second.rotate(p.topLeftCorner<3, 3>());
  cache.second.pretranslate(p.topRightCorner<3, 1>());
  // ret now is cam0--->world, inverse is world---->cam0
  // as in pub method, choose lidar as world
}
template <typename POSE_TYPE>
void read_calib_file(std::ifstream &file, POSE_TYPE &cache, int r, int c){
  std::string line, s;
  std::getline(file, line);
  std::stringstream ss(line);
  POSE_TYPE ret = POSE_TYPE::Identity();
  for(int i = 0; i < r; ++i){
    for(int j = 0; j < c; ++j){
      getline(ss, s, ' ');
      ret(i, j) = std::stod(s);
    }
  }
  cache = ret;
}
// ----------------------------------Data Process end----------------------------------------------------------------------


// ----------------------------------Time Calculate start----------------------------------------------------------------------

template<typename RETURN_TYPE>
struct DateToTimestamp{
  DateToTimestamp(){};
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
      while(cnt < 5 && getline(ss, temp, '-')) data[cnt++] = atof(temp.c_str());
      ss = std::stringstream(strs[1]);
      while(cnt < 5 && getline(ss, temp, ':')) data[cnt++] = atof(temp.c_str());
      getline(ss, temp, ':');
      second_ = static_cast<RETURN_TYPE>(atof(temp.c_str()));
      if(cnt != 5)  return static_cast<RETURN_TYPE>(-1);
    } catch(std::exception e){
      return false;
    }
    std::tm time_tm;
    time_tm.tm_year = data[0] - 1900;
    time_tm.tm_mon = data[1] - 1; // index of month from 0
    time_tm.tm_mday = data[2];
    time_tm.tm_hour = data[3];
    time_tm.tm_min = data[4];
    time_tm.tm_sec = 0; // if float number than add in the tail so it is 0 in there
    time_tm.tm_isdst = -1;
    return static_cast<RETURN_TYPE>(std::mktime(&time_tm)) + second_;
  }
};
// ----------------------------------Time Calculate end----------------------------------------------------------------------
  
} // namespace yan
#endif