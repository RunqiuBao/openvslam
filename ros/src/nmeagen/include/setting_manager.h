#ifndef _SETTING_MANAGER_H_
#define _SETTING_MANAGER_H_


#include <string>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

namespace nmea_convert
{

  struct GgaDefault
  {
    std::string messageID;
    std::string directionLatitude;
    std::string directionLongitude;
    std::string unitOH;
    std::string unitGS;
    int gpsQuality;
    int numSatellites;
    double hdop;
    double orthometricHeight;
    double geoidSeparation;
  };

  struct GsaDefault
  {
    bool outputEnable;
    std::string messageID;
    std::string mode1;
    std::string mode2;
    std::string prn[12];
    double pdop;
    double hdop;
    double vdop;
  };

  struct VtgDefault
  {
    bool outputEnable;
    std::string messageID;
    double trackMadeGoodTrue;
    std::string relativeToTrueNorth;
    double trackMadeGoodMagnetic;
    std::string relativeToMagneticNorth;
    double speedKnots;
    std::string unitKnot;
    double speedKph;
    std::string unitKph;
  };

  struct NmeaDefault
  {
    struct GgaDefault gga;
    struct GsaDefault gsa;
    struct VtgDefault vtg;

  };

  struct SerialSetting
  {
    std::string port;
    int baudrate;
  };

  struct ConvertSetting
  {
    int coordinateNumber;
  };

  struct OutputSetting
  {
    double posErrThreshold;
    double speedEstimation;
    double speed2DEstimation;
    double nmeaOutputRate;
  };

  struct PublisherSetting
  {
    std::string publisherName;
  };

  struct OffsetPosition
  {
    double x;
    double y;
    double z;
    int sec;
    int nsec;
  };

  struct DebugOption
  {
    bool debug;
  };

  struct SettingData
  {
    struct NmeaDefault nmea;
    struct SerialSetting serial;
    struct ConvertSetting conv;
    struct OutputSetting output;
    struct PublisherSetting pub;
    struct OffsetPosition offset;
    struct OffsetPosition bias_offset;
    struct DebugOption debug;
  };

  class SettingManager
  {
    private:
      SettingManager(){};
      ~SettingManager(){};

      bool bias_enable = false;

    public:
      struct SettingData settingData;

      static SettingManager* GetInstance()
      {
        static SettingManager _self;
        return &_self;
      };

      bool LoadFromFile(std::string path);
      bool LoadBiasFile(std::string path);

      void SetDefault();
      bool SaveToFile(std::string path);
  };
}
#endif /*_SETTING_MANAGER_H_*/
