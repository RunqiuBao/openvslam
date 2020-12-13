
#include "coodinate_converter.h"
#include <ros/ros.h>
namespace nmea_convert
{

    CoordinateConverter::CoordinateConverter()
    {
        n = 1.0 / (2*F - 1);

        // α1＝alpha[1]
        alpha[0] = 0;	// 未使用 Indexと合わせるため
        alpha[1] = n/2.0 - 2.0/3.0*std::pow(n, 2) + 5.0/16.0*std::pow(n, 3) + 41.0/180.0*std::pow(n, 4) - 127.0/288.0*std::pow(n, 5);
        alpha[2] = 13.0/48.0*std::pow(n, 2) - 3.0/5.0*std::pow(n, 3) + 557.0/1440.0*std::pow(n, 4) + 281.0/630.0*std::pow(n, 5);
        alpha[3] = 61.0/240.0*std::pow(n, 3) - 103.0/140.0*std::pow(n, 4) + 15061.0/26880.0*std::pow(n, 5);
        alpha[4] = 49561.0/161280.0*std::pow(n, 4) - 179.0/168.0*std::pow(n, 5);
        alpha[5] = 34729.0/80640.0*std::pow(n, 5);

        // beta
        beta[0] = 0;	// 未使用 Indexと合わせるため
        beta[1] = 1.0 / 2.0 * n - 2.0 / 3.0 * std::pow(n, 2) + 37.0 / 96.0 * std::pow(n, 3) - 1.0 / 360.0 * std::pow(n, 4) - 81.0 / 512.0 * std::pow(n, 5);
        beta[2] = 1.0 / 48.0 * std::pow(n, 2) + 1.0 / 15.0 * std::pow(n, 3) - 437.0 / 1440.0 * std::pow(n, 4) + 46.0 / 105.0 * std::pow(n, 5);
        beta[3] = 17.0 / 480.0 * std::pow(n, 3) - 37.0 / 840.0 * std::pow(n, 4) - 209.0 / 4480.0 * std::pow(n, 5);
        beta[4] = 4397.0 / 161280.0 * std::pow(n, 4) - 11.0 / 504.0 * std::pow(n, 5);
        beta[5] = 4583.0 / 161280.0 * std::pow(n, 5);

        // A0...A5
        aa[0] = 1.0 + std::pow(n, 2)/4.0 + std::pow(n, 4)/64.0;
        aa[1] = -3.0/2.0*(n - std::pow(n, 3)/8.0 - std::pow(n, 5)/64.0);
        aa[2] = 15.0/16.0*(std::pow(n, 2) - std::pow(n, 4)/4.0);
        aa[3] = -35.0/48.0*(std::pow(n, 3) - 5.0/16.0*std::pow(n, 5));
        aa[4] = 315.0/512.0*std::pow(n, 4);
        aa[5] = -693.0/1280.0*std::pow(n, 5);

        // δ
        delta[1] = 2.0 * n - 2.0 / 3.0 * std::pow(n, 2) - 2.0 * std::pow(n, 3) + 116.0 / 45.0 * std::pow(n, 4) + 26.0 / 45.0 * std::pow(n, 5) - 2854.0 / 675.0 * std::pow(n, 6);
        delta[2] = 7.0 / 3.0 * std::pow(n, 2) - 8.0 / 5.0 * std::pow(n, 3) - 227.0 / 45.0 * std::pow(n, 4) + 2704.0 / 315.0 * std::pow(n, 5) + 2323.0 / 945.0 * std::pow(n, 6);
        delta[3] = 56.0 / 15.0 * std::pow(n, 3) - 136.0 / 35.0 * std::pow(n, 4) - 1262.0 / 105.0 * std::pow(n, 5) + 73814.0 / 2835.0 * std::pow(n, 6);
        delta[4] = 4279.0 / 630.0 * std::pow(n, 4) - 332.0 / 35.0 * std::pow(n, 5) - 399572.0 / 14175.0 * std::pow(n, 6);
        delta[5] = 4174.0 / 315.0 * std::pow(n, 5) - 144838.0 / 6237.0 * std::pow(n, 6);
        delta[6] = 601676.0 / 22275.0 * std::pow(n, 6);

        // A Bar
        a_bar = ((m0 * a) / (1 + n)) * aa[0];
    }

    /**
     * @fn
     * XY(平面直角座標)から緯度経度へ変換する関数
     * @brief XY(平面直角座標)から緯度経度へ変換する関数
     * @param (double lat0) 対象の平面直角座標系の原点[degree]
     * @param (double lon0) 対象の平面直角座標系の原点[degree]
     * @param (double x) 変換対象のX[m] (東西軸)
     * @param (double y) 変換対象のy[m] (南北軸)
     * @param (double& lat) 緯度[degree]
     * @param (double& lon) 経度[degree]
     * @return 現在のアイテム数
     */
    void CoordinateConverter::ConvertXYToLatLon(int coodinate, double x, double y, double& lat, double& lon)
    {
        double lat0_rad = lat0List[coodinate-1] * M_PI / 180.0;
        double lon0_rad = lon0List[coodinate-1] * M_PI / 180.0;

        s_bar_p0 = 0;
        for(int i=1; i<=5; i++)
        {
            s_bar_p0 = s_bar_p0 + aa[i] * std::sin(2*i*lat0_rad);
        }
        s_bar_p0 = (m0 * a) / (1 + n) * (aa[0]*lat0_rad + s_bar_p0);

        double xi = (x + s_bar_p0) / a_bar;
        double et = y / a_bar;
        double xi_d = 0, et_d = 0, sig_d = 0, tau_d = 0;

        for(int i=1; i<=5; i++)
        {
            xi_d = xi_d + beta[i] * std::sin(2*i*xi) * std::cosh(2*i*et);
            et_d = et_d + beta[i] * std::cos(2*i*xi) * std::sinh(2*i*et);
            sig_d = sig_d + beta[i] * std::cos(2*i*xi) * std::cosh(2*i*et);
            tau_d = tau_d + beta[i] * std::sin(2*i*xi) * std::sinh(2*i*et);
        }
        xi_d = xi - xi_d;
        et_d = et - et_d;
        sig_d = 1 - sig_d;
        double kai = std::asin(std::sin(xi_d) / std::cosh(et_d));

        double lat_tmp, lon_tmp;
        lon_tmp = lon0_rad + std::atan(std::sinh(et_d) / std::cos(xi_d));
        lat_tmp = kai;
        for(int i=1; i<=6; i++)
        {
            lat_tmp = lat_tmp + delta[i] * std::sin(2*i*kai);
        }

        lat = lat_tmp * 180.0 / M_PI;
        lon = lon_tmp * 180.0 / M_PI;
    }
/*
    //双曲線正接関数の逆関数
    double CoordinateConverter::ATanh(double x)
    {
        return (1.0 / 2.0 * std::log((1.0 + x) / (1.0 - x)));
    }
*/
    //-------------------------------------------------------
    // ジオイドファイルを読み込む（配列にジオイドデータを取り込む）
    int CoordinateConverter::ReadGeoid2000BinFile2(std::string path)
    {
        int i, j, M, N, t;
        double dH;

        nGeoid2000.resize(MAX_GEO2000_LAT);
        for(int k=0; k<MAX_GEO2000_LAT; k++)
        {
            nGeoid2000[k].resize(MAX_GEO2000_LON);
        }

        std::ifstream fin( path.c_str(), std::ios::in | std::ios::binary );
        if (!fin){
            return -1;
        }

        while(!fin.eof()){  //ファイルの最後まで続ける
            fin.read( ( char * ) &dH, sizeof( double ) );
            fin.read( ( char * ) &i, sizeof( int ) );
            fin.read( ( char * ) &j, sizeof( int ) );
            //文字列ではないデータを読みこむ
            ROS_DEBUG("Geoid[%d][%d] = %f", i, j, dH);
            M = i - 240;
            N = j - 120;
            if ((M >= 0 && M < MAX_GEO2000_LAT) && (N >= 0 && N < MAX_GEO2000_LON))
                nGeoid2000.at(M).at(N) = (int)(dH * 10000);     // ジオイド高を取得
        }
        fin.close();  //ファイルを閉じる

        return 0;
    }

    //--------------------------------------------------------------------------------------------------------------
    // GGAの経緯度から、ジオイドモデル2011より現在位置におけるジオイド高を求める。
    void CoordinateConverter::CalculateGeoidFile2(double dLat1, double dLon1, double dElpH, double& dGeoidH, double& dH)
    {
        if(nGeoid2000.size() < 1)
        {
            dGeoidH = 0;
            dH = 0;
            return;
        } 
        /* 引数：
            dLat1, dLon1は緯度、経度で単位は°です。
            dElpHはGGAに含まれる2種類の高さを足した楕円体高(m)です。
            求めたい位置の経緯度と楕円体高を入力すると、その地点のジオイド高と標高が戻り値として返ります。
        */
        int nX1, nY1, nX2, nY2, nX3, nY3, nX4, nY4;
        double dX1, dY1, dX2, dY2, dMeshRot, dL;
        double dNowX, dNowY;
        double dLatMeshSize, dLonMeshSize;
        double dX[4];
        double dY[4];
        int M1, M2, M3, M4, N1, N2, N3, N4;

        dX1 = 20.0;                                 // ジオイドメッシュ開始点
        dY1 = 120.0;
        dX2 = 50.0;                                 // 緯度MAX点
        dY2 = 120.0;
        dL = 150.0;
        dMeshRot = std::atan2(dY2 - dY1, dX2 - dX1);         // メッシュ座標の方向角


        dLatMeshSize = 0.016667;                // 緯度1分間隔メッシュ（°単位）
        dLonMeshSize = 0.025;                   // 経度1分間隔メッシュ（°単位）

        //--------------- 現在地点を包括する四隅のメッシュインデックスを求める --------------------
        dMeshRot *= (180.0 / M_PI);
        ChangeXYtoMesh(dX1, dY1, dMeshRot, dLat1, dLon1, dNowX, dNowY);


        // メッシュ座標は縦方向をX座標・横方向をY座標とする
        //											|	
        //										  X	|   点2 _________点3
        //											|	   |    ｜    |
        //											|	   |    ｜    |
        //											|	   |----●----|
        //											|	   |____｜____|
        //											|   点1  		 点4
        //											|_______________________ Y
        //										   O
        nX1 = (int)std::floor(dNowX / dLatMeshSize);
        nY1 = (int)std::floor(dNowY / dLonMeshSize);
        nX2 = nX1 + 1;
        nY2 = nY1;
        nX3 = nX1 + 1;
        nY3 = nY1 + 1;
        nX4 = nX1;
        nY4 = nY1 + 1;
        //-----------------------------------------------------------------------------------------

        // 限定されたメッシュの中に当てはまるみたい
        if (!(((nX1 >= 0 && nX1 < MAX_GEO2000_LAT) && (nY1 >= 0 && nY1 < MAX_GEO2000_LON)) &&
            ((nX3 >= 0 && nX3 < MAX_GEO2000_LAT) && (nY3 >= 0 && nY3 < MAX_GEO2000_LON))))
        {
            dH = -9999.9;
            dGeoidH = -9999.9;
            return;
        }


        //--------------- 四隅のメッシュ座標から現在地点のジオイド高を求める ----------------------
        //										    X
        //											2  _________________ 3
        //											  |				    | 
        //											  |					|
        //										   C  |------・---------| D
        //											  |		 P 			|
        //											  |       		　  |
        //											1 |_________________| 4  Y
        //
        //						線分12の距離 = L12				線分1Cの距離 = L1c
        //						線分34の距離 = L34				線分4Dの距離 = L4d
        //						線分CDの距離 = Lcd				線分CPの距離 = Lcp
        //						各頂点の高さ	H1, H2, H3, H4, Hc, Hd　とすると求点Hpは
        //						
        //						Hc = H1 + (H2-H1)*L1c / L12 ------- ①
        //						Hd = H4 + (H3-H4)*L4d / L34 ------- ②
        //						Hp = Hc + (Hd-Hc)*Lcp / Lcd ------- ③
        //						①～③の補間式により点Pの高さHpが求まる

        // (X, Y)は測量軸とする
        double dHc, dHd, dHp;
        double a, b;
        double dL1c, dLcp;
        double dH1, dH2, dH3, dH4;

        ChangeMeshtoXY(dX1, dY1, dMeshRot, nX1 * dLatMeshSize, nY1 * dLonMeshSize, dX[0], dY[0]);
        ChangeMeshtoXY(dX1, dY1, dMeshRot, nX2 * dLatMeshSize, nY2 * dLonMeshSize, dX[1], dY[1]);
        ChangeMeshtoXY(dX1, dY1, dMeshRot, nX3 * dLatMeshSize, nY3 * dLonMeshSize, dX[2], dY[2]);
        ChangeMeshtoXY(dX1, dY1, dMeshRot, nX4 * dLatMeshSize, nY4 * dLonMeshSize, dX[3], dY[3]);


        M1 = nX1 - 240;             // 北緯24°～46°まで、東経123°～147°までのメッシュ座標に変換する
        N1 = nY1 - 120;


        M2 = M1 + 1;
        N2 = N1;
        M3 = M1 + 1;
        N3 = N1 + 1;
        M4 = M1;
        N4 = N1 + 1;
        dH1 = nGeoid2000.at(M1).at(N1) / 10000.0;
        dH2 = nGeoid2000.at(M2).at(N2) / 10000.0;
        dH3 = nGeoid2000.at(M3).at(N3) / 10000.0;
        dH4 = nGeoid2000.at(M4).at(N4) / 10000.0;

        // 点1・点4を通る直線の式
        a = dX[3] - dX[0];
        b = dY[3] - dY[0];
        // 点(dMeshX, dMeshY)と線分14との距離dL1c
        dL1c = std::abs(-b * dLat1 + a * dLon1 + b * dX[0] - a * dY[0]) / std::sqrt(a * a + b * b);

        // 点1・点2を通る直線の式
        a = dX[1] - dX[0];
        b = dY[1] - dY[0];
        // 点(dMeshX, dMeshY)と線分12との距離dLcp
        dLcp = std::abs(-b * dLat1 + a * dLon1 + b * dX[0] - a * dY[0]) / std::sqrt(a * a + b * b);

        // ①式より点Cの高さを求める	{ Hc = H1 + (H2-H1)*L1c / L12 }
        dHc = dH1 + (((dH2 - dH1) * dL1c) / std::sqrt(std::pow(dX[1] - dX[0], 2.0) + std::pow(dY[1] - dY[0], 2.0)));

        // ②式より点Dの高さを求める	{ Hd = H4 + (H3-H4)*L4d / L34 }
        dHd = dH4 + (((dH3 - dH4) * dL1c) / std::sqrt(std::pow(dX[2] - dX[3], 2.0) + std::pow(dY[2] - dY[3], 2.0)));

        // ③式より点Pのジオイド高を求める	{ Hp = Hc + (Hd-Hc)*Lcp / Lcd }
        dHp = dHc + (((dHd - dHc) * dLcp) / std::sqrt(std::pow(dX[3] - dX[0], 2.0) + std::pow(dY[3] - dY[0], 2.0)));
        //---------------------------------------------------------------------------------

        dGeoidH = dHp;         // ジオイド高
        dH = dElpH - dHp;      // 標高を求める（WGS-84系楕円体高 - WGS-84系ジオイド高）
    }

    //-------------------------------------------------------------------------------------------------------
    void CoordinateConverter::ChangeXYtoMesh(double dX0, double dY0, double dRot, double dX1, double dY1, double& dRotX, double& dRotY)
    {
        double dDeltaX, dDeltaY;

        dDeltaX = dX1 - dX0;
        dDeltaY = dY1 - dY0;

        dRot *= (M_PI / 180.0);

        dRotX = dDeltaX * std::cos(dRot) + dDeltaY * std::sin(dRot);
        dRotY = -dDeltaX * std::sin(dRot) + dDeltaY * std::cos(dRot);
    }


    void CoordinateConverter::ChangeMeshtoXY(double dX0, double dY0, double dRot, double dX1, double dY1, double& dRotX, double& dRotY)
    {
        double dDeltaX, dDeltaY;

        dDeltaX = dX1;
        dDeltaY = dY1;

        dRot *= (M_PI / 180.0);

        dRotX = dDeltaX * std::cos(dRot) - dDeltaY * std::sin(dRot) + dX0;
        dRotY = dDeltaX * std::sin(dRot) + dDeltaY * std::cos(dRot) + dY0;
    }
}

