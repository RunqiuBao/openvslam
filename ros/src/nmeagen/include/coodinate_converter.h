#ifndef _COODINATE_CONVERTER_H_
#define _COODINATE_CONVERTER_H_

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

#include <iostream>
#include <fstream>

namespace nmea_convert
{
    class CoordinateConverter
    {
        public:
            void ConvertXYToLatLon(int coodinate, double x, double y, double& lat, double& lon);

            int ReadGeoid2000BinFile2(std::string path);
            void CalculateGeoidFile2(double dLat1, double dLon1, double dElpH, double& dGeoidH, double& dH);
            void ChangeXYtoMesh(double dX0, double dY0, double dRot, double dX1, double dY1, double& dRotX, double& dRotY);
            void ChangeMeshtoXY(double dX0, double dY0, double dRot, double dX1, double dY1, double& dRotX, double& dRotY);

        private:
            CoordinateConverter();
            ~CoordinateConverter(){};

            // 座標変換に必要なパラメータ
            const double a = 6378137;           //! 長半径
            const double F = 298.257222101;	    //! 逆扁平率
            const double m0 = 0.9999;           //! 縮尺係数
            
            const double lat0List[20] = {
              33.0000, 33.0000, 36.0000, 33.0000, 36.0000,
              36.0000, 36.0000, 36.0000, 36.0000, 40.0000,
              44.0000, 44.0000, 44.0000, 26.0000, 26.0000,
              26.0000, 26.0000, 20.0000, 26.0000};

            const double lon0List[20] = {
              129.0 + (30.0 / 60.0), 131.0000, 132.0 + (10.0 / 60.0), 133.0 + (30.0 / 60.0), 134.0 + (20.0 / 60.0),
              136.0000, 137.0 + (10.0 / 60.0), 138.0 + (30.0 / 60.0), 139.0 + (50.0 / 60.0), 140.0 + (50.0 / 60.0),
              140.0 + (15.0 / 60.0), 142.0 + (15.0 / 60.0), 144.0 + (15.0 / 60.0), 142.0000, 127.0 + (30.0 / 60.0),
              124.0000, 131.0000, 136.0000, 154.0000};

            double n;
            double alpha[6];
            double beta[6];
            double delta[7];
            double aa[6];
            double a_bar;
            double s_bar_p0;

            // ジオイド高に必要なパラメータ
            const int MAX_GEO2000_LAT = 1801;
            const int MAX_GEO2000_LON = 1201;

            std::vector<std::vector<int>> nGeoid2000;


            //static double CoordinateConverter::ATanh(double x);
        public:
            static CoordinateConverter& GetInstance()
            {
                static CoordinateConverter _instance;
                return _instance;
            }
    };
}

#endif /*_COODINATE_CONVERTER_H_*/