#ifndef _SEND_NMEA_H_
#define _SEND_NMEA_H_

#include <ros/ros.h>
#include "serial_controller.h"
#include "topic_vector.h"

namespace nmea_convert
{
  class SendNmea
  {

    public:
      SendNmea(ros::NodeHandle& node, TopicVector* p_topic_vec, SettingData* p_setting);
      ~SendNmea(){};

      // NMEA送信開始
      void SendEnable(bool enable);
      void TimerCallback(const ros::TimerEvent& e);

    private:
      //! nodeオブジェクト
      ros::NodeHandle* _nh_p;
      
      //! Timerオブジェクト
      ros::Timer _timer;
      
      //! NMEAデフォルト値(設定ファイル)
      NmeaDefault _nmeaDef_set;
      OutputSetting _output_set;
      ConvertSetting _convert_set;

      //! 送信用Serialオブジェクト
      SerialController* _serial = nullptr;

      //! トピックVector
      TopicVector* _topic_p;

      //! NMEA生成ロジック
      bool CalcCheckSum(std::string nmea, unsigned char& check_sum);

  };
}

#endif /*_SEND_NMEA_H_*/