/**
 * @file serial_controller.cpp
 * @brief Serial通信をコントロールするクラス
 * @author ISL
 * 
 */

#include "serial_controller.h"
#include <termios.h>

namespace nmea_convert
{
    SerialController::SerialController(SerialSettingData& data)
    {
        _data = data;
    }

    SerialController::~SerialController()
    {
        if(_fd > 0) Close();
    }

    int SerialController::Open()
    {
        struct termios tio;
        
        _fd = open(_data._path.c_str(), O_RDWR);
        if(_fd < 0){
            ROS_ERROR("%s doesn't open it.(%d)\n",_data._path.c_str(), _fd);
            return -1;
        }

	    memset(&tio,0,sizeof(tio));
        tio.c_cflag = CS8 | CLOCAL | CREAD;

        // Baudrate変換
        int baudrate;
        switch(_data._baudrate)
        {
            case 9600:
                baudrate = B9600;
                break;
            case 19200:
                baudrate = B19200;
                break;
            case 38400:
                baudrate = B38400;
                break;
            case 57600:
                baudrate = B57600;
                break;
            case 115200:
                baudrate = B115200;
                break;
            case 230400:
                baudrate = B230400;
                break;
            default:
                baudrate = B9600;
                break;
        }
        cfsetispeed(&tio, baudrate);
        cfsetospeed(&tio, baudrate);
        tcsetattr(_fd,TCSANOW,&tio);


        return 0;
    }

    int SerialController::Close()
    {
        close(_fd);
        return 0;
    }

    int SerialController::Write(std::string message)
    {
        return write(_fd, message.c_str() , message.length());
    }
}
