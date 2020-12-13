#include "setting_manager.h"
#include "ros/ros.h"

#include <iostream>
#include <fstream>
#include <list>
#include <boost/algorithm/string.hpp>
#include "algorithm"

using namespace boost::property_tree;
namespace nmea_convert
{

  bool SettingManager::LoadFromFile(std::string path)
  {
    ptree pt;

    SetDefault();
    // 存在チェック
    // ファイル読み込み(boost使用)
    try
    {
      read_ini(path, pt);
    }
    catch(boost::property_tree::ini_parser::ini_parser_error &ex)
    {
      // ファイルフォーマットエラー
      return false;
    }

    // 設定反映(デフォルト値つき)
    settingData.nmea.gga.messageID = pt.get("NMEA-GGA.MessageID", "$GPGGA");
    settingData.nmea.gga.directionLatitude = pt.get("NMEA-GGA.DirectionLatitude", "N");
    settingData.nmea.gga.directionLongitude = pt.get("NMEA-GGA.DirectionLongitude", "E");
    settingData.nmea.gga.gpsQuality = pt.get("NMEA-GGA.GPSQuality", 4);
    settingData.nmea.gga.numSatellites = pt.get("NMEA-GGA.NumSatelites", 12);
    settingData.nmea.gga.hdop = pt.get("NMEA-GGA.HDOP", 1.0f);
    settingData.nmea.gga.orthometricHeight = pt.get("NMEA-GGA.OrthometricHeight", 0.0f);
    settingData.nmea.gga.unitOH = pt.get("NMEA-GGA.UnitOH", "M");
    settingData.nmea.gga.geoidSeparation = pt.get("NMEA-GGA.GeoidSeparation", 0.0f);
    settingData.nmea.gga.unitGS = pt.get("NMEA-GGA.UnitGS", "M");

    settingData.nmea.gsa.messageID = pt.get("NMEA-GSA.MessageID", "$GPGSA");
    settingData.nmea.gsa.outputEnable = pt.get("NMEA-GSA.OutputEnable", true);
    settingData.nmea.gsa.mode1 = pt.get("NMEA-GSA.Mode1", "A");
    settingData.nmea.gsa.mode2 = pt.get("NMEA-GSA.Mode2", "3");
    for(int i=0; i<12; i++)
    {
      std::string key_string;
      key_string = "NMEA-GSA.Prn" + std::to_string(i);
      settingData.nmea.gsa.prn[i] = pt.get(key_string, "");
    }
    settingData.nmea.gsa.pdop = pt.get("NMEA-GSA.PDOP", 1.0f);
    settingData.nmea.gsa.hdop = pt.get("NMEA-GSA.HDOP", 1.0f);
    settingData.nmea.gsa.vdop = pt.get("NMEA-GSA.VDOP", 1.0f);

    settingData.nmea.vtg.messageID = pt.get("NMEA-VTG.MessageID", "$GPVTG");
    settingData.nmea.vtg.outputEnable = pt.get("NMEA-VTG.OutputEnable", true);
    settingData.nmea.vtg.trackMadeGoodTrue = pt.get("NMEA-VTG.TrackMadeGoodTrue", 0.0f);
    settingData.nmea.vtg.relativeToTrueNorth = pt.get("NMEA-VTG.RelativeToTrueNorth", "T");
    settingData.nmea.vtg.trackMadeGoodMagnetic = pt.get("NMEA-VTG.TrackMadeGoodMagnetic", 0.0f);
    settingData.nmea.vtg.relativeToMagneticNorth = pt.get("NMEA-VTG.RelativeToMagneticNorth", "M");
    settingData.nmea.vtg.speedKnots = pt.get("NMEA-VTG.SpeedKnots", 0.0f);
    settingData.nmea.vtg.speedKph = pt.get("NMEA-VTG.SpeedKph", 0.0f);
    settingData.nmea.vtg.unitKnot = pt.get("NMEA-VTG.UnitKnot", "N");
    settingData.nmea.vtg.unitKph = pt.get("NMEA-VTG.UnitKph", "K");

    settingData.serial.port = pt.get("Serial.port", "/dev/ttyUSB0");
    settingData.serial.baudrate = pt.get("Serial.baudrate", 9600);

    settingData.conv.coordinateNumber = pt.get("Coordinate.Number", 9);

    settingData.output.posErrThreshold = pt.get("Threshold.PosErrThreshold", 10.0);
    settingData.output.speedEstimation = pt.get("Threshold.SpeedEstimation", 3);
    settingData.output.speed2DEstimation = pt.get("Threshold.2DSpeedEstimation", 3);
    settingData.output.nmeaOutputRate = pt.get("Rate.NmeaRate", 5);
    
    settingData.pub.publisherName = pt.get("Publisher.PublisherName", "traj");

    settingData.offset.x = pt.get("Offset.X", 0.0);
    settingData.offset.y = pt.get("Offset.Y", 0.0);
    settingData.offset.z = pt.get("Offset.Z", 0.0);
    settingData.offset.sec = pt.get("Offset.Sec", 0);
    settingData.offset.nsec = pt.get("Offset.NSec", 0);

    settingData.debug.debug = pt.get("Debug.Debug", false);
    return true;
  }

  void SettingManager::SetDefault()
  {
    settingData.nmea.gga.messageID = "$GPGGA";
    settingData.nmea.gga.directionLatitude = "N";
    settingData.nmea.gga.directionLongitude = "E";
    settingData.nmea.gga.gpsQuality = 4;
    settingData.nmea.gga.numSatellites = 12;
    settingData.nmea.gga.hdop = 1.0f;
    settingData.nmea.gga.orthometricHeight = 0.0f;
    settingData.nmea.gga.unitOH = "M";
    settingData.nmea.gga.geoidSeparation = 0.0f;
    settingData.nmea.gga.unitGS = "M";

    settingData.nmea.gsa.messageID = "$GPGSA";
    settingData.nmea.gsa.outputEnable = true;
    settingData.nmea.gsa.mode1 = "A";
    settingData.nmea.gsa.mode2 = "3";
    for(int i=0; i<12; i++)
    {
      settingData.nmea.gsa.prn[i] = "";
    }
    settingData.nmea.gsa.pdop = 1.0f;
    settingData.nmea.gsa.hdop = 1.0f;
    settingData.nmea.gsa.vdop = 1.0f;

    settingData.nmea.vtg.messageID = "$GPVTG";
    settingData.nmea.vtg.outputEnable = true;
    settingData.nmea.vtg.trackMadeGoodTrue = 0.0f;
    settingData.nmea.vtg.relativeToTrueNorth = "T";
    settingData.nmea.vtg.trackMadeGoodMagnetic = 0.0f;
    settingData.nmea.vtg.relativeToMagneticNorth = "M";
    settingData.nmea.vtg.speedKnots = 0.0f;
    settingData.nmea.vtg.unitKnot = "N";
    settingData.nmea.vtg.speedKph = 0.0f;
    settingData.nmea.vtg.unitKph = "K";

    settingData.serial.port = "/dev/ttyUSB0";
    settingData.serial.baudrate = 9600;

    settingData.conv.coordinateNumber = 9;

    settingData.output.posErrThreshold = 10.0f;
    settingData.output.speedEstimation = 3;
    settingData.output.speed2DEstimation = 3;
    settingData.output.nmeaOutputRate = 5;
    
    settingData.pub.publisherName = "traj";

    settingData.offset.x = 0.0f;
    settingData.offset.y = 0.0f;
    settingData.offset.z = 0.0f;
    settingData.offset.sec = 0;
    settingData.offset.nsec = 0;

    settingData.debug.debug = false;
  }

  bool SettingManager::SaveToFile(std::string path)
  {
    ptree pt;

    pt.put("NMEA-GGA.MessageID", settingData.nmea.gga.messageID);
    pt.put("NMEA-GGA.DirectionLatitude", settingData.nmea.gga.directionLatitude);
    pt.put("NMEA-GGA.DirectionLongitude", settingData.nmea.gga.directionLongitude);
    pt.put("NMEA-GGA.GPSQuality", settingData.nmea.gga.gpsQuality);
    pt.put("NMEA-GGA.NumSatelites", settingData.nmea.gga.numSatellites);
    pt.put("NMEA-GGA.HDOP", settingData.nmea.gga.hdop);
    pt.put("NMEA-GGA.OrthometricHeight", settingData.nmea.gga.orthometricHeight);
    pt.put("NMEA-GGA.UnitOH", settingData.nmea.gga.unitOH);
    pt.put("NMEA-GGA.GeoidSeparation", settingData.nmea.gga.geoidSeparation);
    pt.put("NMEA-GGA.UnitGS", settingData.nmea.gga.unitGS);

    pt.put("NMEA-GSA.MessageID", settingData.nmea.gsa.messageID);
    pt.put("NMEA-GSA.OutputEnable", settingData.nmea.gsa.outputEnable);
    pt.put("NMEA-GSA.Mode1", settingData.nmea.gsa.mode1);
    pt.put("NMEA-GSA.Mode2", settingData.nmea.gsa.mode2);
    for(int i=0; i<12; i++)
    {
      std::string key_string;
      key_string = "NMEA-GSA.Prn" + std::to_string(i);
      pt.put(key_string, settingData.nmea.gsa.prn[i]);
    }
    pt.put("NMEA-GSA.PDOP", settingData.nmea.gsa.pdop);
    pt.put("NMEA-GSA.HDOP", settingData.nmea.gsa.hdop);
    pt.put("NMEA-GSA.VDOP", settingData.nmea.gsa.vdop);

    pt.put("NMEA-VTG.MessageID", settingData.nmea.vtg.messageID);
    pt.put("NMEA-VTG.OutputEnable", settingData.nmea.vtg.outputEnable);
    pt.put("NMEA-VTG.TrackMadeGoodTrue", settingData.nmea.vtg.trackMadeGoodTrue);
    pt.put("NMEA-VTG.RelativeToTrueNorth", settingData.nmea.vtg.relativeToTrueNorth);
    pt.put("NMEA-VTG.TrackMadeGoodMagnetic", settingData.nmea.vtg.trackMadeGoodMagnetic);
    pt.put("NMEA-VTG.RelativeToMagneticNorth", settingData.nmea.vtg.relativeToMagneticNorth);
    pt.put("NMEA-VTG.SpeedKnots", settingData.nmea.vtg.speedKnots);
    pt.put("NMEA-VTG.SpeedKph", settingData.nmea.vtg.speedKph);
    pt.put("NMEA-VTG.UnitKnot", settingData.nmea.vtg.unitKnot);
    pt.put("NMEA-VTG.UnitKph", settingData.nmea.vtg.unitKph);

    pt.put("Serial.port", settingData.serial.port);
    pt.put("Serial.baudrate", settingData.serial.baudrate);

    pt.put("Coordinate.Number", settingData.conv.coordinateNumber);

    pt.put("Threshold.PosErrThreshold", settingData.output.posErrThreshold);
    pt.put("Threshold.SpeedEstimation", settingData.output.speedEstimation);
    pt.put("Threshold.2DSpeedEstimation", settingData.output.speed2DEstimation);
    pt.put("Rate.NmeaRate", settingData.output.nmeaOutputRate);

    pt.put("Publisher.PublisherName", settingData.pub.publisherName);

    pt.put("Offset.X", settingData.offset.x);
    pt.put("Offset.Y", settingData.offset.y);
    pt.put("Offset.Z", settingData.offset.z);
    pt.put("Offset.Sec", settingData.offset.sec);
    pt.put("Offset.NSec", settingData.offset.nsec);

    pt.put("Debug.Debug", settingData.debug.debug);

   write_ini(path, pt);
  }

  bool SettingManager::LoadBiasFile(std::string path)
  {
    // File Open
    std::ifstream ifs(path);
    if(!ifs)
    {
      ROS_ERROR("Bias file open error.(%s)", path.c_str());
      return false;
    }

    try{
      // 1行読み込み
      std::string buff, name, value;
      std::list<std::string> list_string;
      while(std::getline(ifs, buff))
      {
        ROS_DEBUG("Bias file read%s", buff.c_str());
        //trim
        boost::trim (buff);
        // コメントチェック
        if(buff[0] == '#') continue;
        // デリミタで分割
        boost::split(list_string, buff, boost::is_any_of(":"));
        // 名前取得
        auto it = list_string.begin();
        name = *(it);
        std::transform(name.begin(), name.end(), name.begin(), 
          [](unsigned char c) -> unsigned char { return std::tolower(c); });

        // 値取得
        it++;
        value = *(it);
        ROS_DEBUG("Bias file %s=%s", name.c_str(), value.c_str());
        if(name == "x")
        {
          settingData.bias_offset.x = stod(value);
        }
        else if(name == "y")
        {
          settingData.bias_offset.y = stod(value);
        }
        else if(name == "z")
        {
          settingData.bias_offset.z = stod(value);
        }
        else if(name == "sec")
        {
          settingData.bias_offset.sec = stoi(value);
        }
        else if(name == "nsec")
        {
          settingData.bias_offset.nsec = stoi(value);
        }
      }
    }
    catch(const std::invalid_argument& e)
    {
      ROS_ERROR("Bias file invalid_argument.(%s)\n%s", path.c_str(), e.what());
      return false;
    }
    catch(const std::out_of_range& e)
    {
      ROS_ERROR("Bias file out_of_range.(%s)", path.c_str());
      return false;
    }
    catch(const std::exception& e)
    {
      ROS_ERROR("Bias file exception.(%s)", e.what());
      return false;
   }

    bias_enable = true;
    return true;
  }


}