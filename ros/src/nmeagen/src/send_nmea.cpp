#include "send_nmea.h"
#include "coodinate_converter.h"
#include <boost/format.hpp>

namespace nmea_convert
{
  SendNmea::SendNmea(ros::NodeHandle& node, TopicVector* p_topic_vec, SettingData* p_setting)
  {
    _nh_p = &node;
    SerialSettingData serial;
    serial._path = p_setting->serial.port;
    serial._baudrate = p_setting->serial.baudrate;
    _serial = new SerialController(serial);

    _nmeaDef_set = p_setting->nmea;
    _output_set = p_setting->output;
    _convert_set = p_setting->conv;

    _topic_p = p_topic_vec;
  }

  void SendNmea::SendEnable(bool enable)
  {
    if(enable)
    {
      // Timer有効にする。元仕掛けていたタイマーは無効とする
      if(_timer.isValid())
      {
        _timer.stop();
      }

      double rate = 1.0/_output_set.nmeaOutputRate;
      
      _serial->Open();
      _timer = _nh_p->createTimer(ros::Duration(rate), &SendNmea::TimerCallback, this);
      ROS_INFO("valid(%d),start(%d)", _timer.isValid(), _timer.hasStarted());
    }
    else
    {
      if(_timer.isValid())
      {
        _timer.stop();
      }
    }
    
  }

  void SendNmea::TimerCallback(const ros::TimerEvent& e)
  {
    // TopicVectorからデータ取得
    struct topic_st item;

    if(_topic_p->GetPropergateItem(item, _output_set.speedEstimation, ros::Time::now()))
    {

      // 座標変換
      double lat, lon;
      CoordinateConverter::GetInstance().ConvertXYToLatLon(_convert_set.coordinateNumber, item.x, item.y, lat, lon);
      ROS_DEBUG("Convert: %f,%f>>>%f,%f", item.x, item.y, lat, lon);

      // Geoid高取得
      double geoidH, height;
      CoordinateConverter::GetInstance().CalculateGeoidFile2(lat, lon, 0, geoidH, height);
      if(geoidH == 0.0 || geoidH == -9999.9)
      {
        // 変換失敗
        height = item.z;
        geoidH = _nmeaDef_set.gga.geoidSeparation;
      }
      else
      {
        height = item.z;
      }

      // GGA生成
      char date[100];
      time_t t = item.daytime.sec;
      strftime(date, sizeof(date), "%H%M%S", localtime(&t));

      int lat_deg = std::floor(lat);
      int lat_min = std::floor((lat - lat_deg) * 60.0);
      int lat_min_dot = std::round(((lat - lat_deg) * 60.0 - lat_min) * 10000);
      if(lat_min_dot >= 10000)
      {
        lat_min_dot -= 10000;
        lat_min++;
      }
      if(lat_min >= 60)
      {
        lat_min -= 60;
        lat_deg++;
      }

      int lon_deg = std::floor(lon);
      int lon_min = std::floor((lon - lon_deg) * 60.0);
      int lon_min_dot = std::round(((lon - lon_deg) * 60.0 - lon_min) * 10000);
      if(lon_min_dot >= 10000)
      {
        lon_min_dot -= 10000;
        lon_min++;
      }
      if(lon_min >= 60)
      {
        lon_min -= 60;
        lon_deg++;
      }

      std::string nmea_gga = (boost::format("%s,%s.%03d,%02d%02d.%04d,%s,%03d%02d.%04d,%s,%d,%02d,%.1f,%.4f,%s,%.4f,%s,") 
        % _nmeaDef_set.gga.messageID.c_str()
        % date % std::floor(item.daytime.nsec / 1000 / 1000)
        % lat_deg % lat_min % lat_min_dot % _nmeaDef_set.gga.directionLatitude.c_str()
        % lon_deg % lon_min % lon_min_dot % _nmeaDef_set.gga.directionLongitude.c_str()
        % _nmeaDef_set.gga.gpsQuality
        % _nmeaDef_set.gga.numSatellites
        % _nmeaDef_set.gga.hdop
        % height % _nmeaDef_set.gga.unitOH.c_str()
        % geoidH % _nmeaDef_set.gga.unitGS.c_str()
      ).str();
      nmea_gga += ",";
      nmea_gga += "*";

      // チェックサム計算
      unsigned char check_sum = 0;
      CalcCheckSum(nmea_gga, check_sum);
      nmea_gga += (boost::format("%02X") % (unsigned int)check_sum).str();
      nmea_gga += "\r\n";
      _serial->Write(nmea_gga);
      ROS_INFO("NMEA:%s", nmea_gga.c_str());

      if(_nmeaDef_set.gsa.outputEnable)
      {
        // GSA生成
        std::string nmea_gsa = (boost::format("%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%.1f,%.1f,%.1f") 
          % _nmeaDef_set.gsa.messageID.c_str()
          % _nmeaDef_set.gsa.mode1.c_str()
          % _nmeaDef_set.gsa.mode2.c_str()
          % _nmeaDef_set.gsa.prn[0].c_str()
          % _nmeaDef_set.gsa.prn[1].c_str()
          % _nmeaDef_set.gsa.prn[2].c_str()
          % _nmeaDef_set.gsa.prn[3].c_str()
          % _nmeaDef_set.gsa.prn[4].c_str()
          % _nmeaDef_set.gsa.prn[5].c_str()
          % _nmeaDef_set.gsa.prn[6].c_str()
          % _nmeaDef_set.gsa.prn[7].c_str()
          % _nmeaDef_set.gsa.prn[8].c_str()
          % _nmeaDef_set.gsa.prn[9].c_str()
          % _nmeaDef_set.gsa.prn[10].c_str()
          % _nmeaDef_set.gsa.prn[11].c_str()
          % _nmeaDef_set.gsa.pdop
          % _nmeaDef_set.gsa.hdop
          % _nmeaDef_set.gsa.vdop
        ).str();
        nmea_gsa += "*";

        // チェックサム計算
        check_sum = 0;
        CalcCheckSum(nmea_gsa, check_sum);
        nmea_gsa += (boost::format("%02X") % (unsigned int)check_sum).str();
        nmea_gsa += "\r\n";

        _serial->Write(nmea_gsa);
        ROS_INFO("NMEA:%s", nmea_gsa.c_str());
      }

      if(_nmeaDef_set.vtg.outputEnable)
      {
        // VTG生成
        double speed;
        if(!_topic_p->GetSpeedKph(speed, _output_set.speed2DEstimation)) speed = _nmeaDef_set.vtg.speedKph;
        std::string nmea_vtg = (boost::format("%s,%.4f,%s,%.4f,%s,%.4f,%s,%.4f,%s") 
          % _nmeaDef_set.vtg.messageID.c_str()
          % _nmeaDef_set.vtg.trackMadeGoodTrue % _nmeaDef_set.vtg.relativeToTrueNorth.c_str()
          % _nmeaDef_set.vtg.trackMadeGoodMagnetic % _nmeaDef_set.vtg.relativeToMagneticNorth.c_str()
          % _nmeaDef_set.vtg.speedKnots % _nmeaDef_set.vtg.unitKnot.c_str()
          % speed % _nmeaDef_set.vtg.unitKph.c_str()
        ).str();
        nmea_vtg += "*";

        // チェックサム計算
        check_sum = 0;
        CalcCheckSum(nmea_vtg, check_sum);
        nmea_vtg += (boost::format("%02X") % (unsigned int)check_sum).str();
        nmea_vtg += "\r\n";
        _serial->Write(nmea_vtg);
        ROS_INFO("NMEA:%s", nmea_vtg.c_str());
      }
    }
  }

  bool SendNmea::CalcCheckSum(std::string nmea, unsigned char& check_sum)
  {
    int start_pos = nmea.find('$');
    int end_pos = nmea.find('*');
    unsigned char sum = 0;

    if(start_pos == -1 || end_pos == -1) return false;

    for(int i=start_pos+1; i<end_pos; i++)
    {
      sum ^= (unsigned char)(nmea[i]);
    }

    check_sum = sum;

    return true;
  }
}
