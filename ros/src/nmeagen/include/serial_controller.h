#ifndef _SERIAL_CONTROLLER_H_
#define _SERIAL_CONTROLLER_H_

/**
 * @file serial_controller.cpp
 * @brief Serial通信をコントロールするクラス
 * @author ISL
 * 
 */

#include <string>
#include <sys/ioctl.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <ros/ros.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

namespace nmea_convert
{
  class SerialSettingData
  {
    public:
      std::string _path;
      int _baudrate;
  };

  class SerialController
  {
    private:
      // Serialオブジェクト
      int _fd = INT_MIN;
      // SettingData
      SerialSettingData _data;

    public:
      SerialController(SerialSettingData& data);
      ~SerialController();

      int Open();
      int Close();
      int Write(std::string message);
  };
}

#endif /*_SERIAL_CONTROLLER_H_*/