#ifndef _TOPIC_VECTOR_H_
#define _TOPIC_VECTOR_H_


#include <mutex>
#include <vector>
#include <sys/time.h>
#include <ros/ros.h>
#include "setting_manager.h"

namespace nmea_convert
{
    struct topic_st
    {
        //! 公共座標
        double x;
        double y;
        double z;

        //! 日時データ
        ros::Time daytime;
    };

    class TopicVector{
        public:
            TopicVector(int size, nmea_convert::OutputSetting& output);
            ~TopicVector(){};

            int SetItem(const struct topic_st& src_item);
            bool GetItem(struct topic_st& output_item);
            bool GetPropergateItem(struct topic_st& output_item, int num_pos, ros::Time now);
            bool GetSpeedKph(double& output_speed, int num_pos);

        private:
            std::recursive_mutex _mutex;
            std::vector<struct topic_st> _vc;
            int _size;
            nmea_convert::OutputSetting* _p_outputSetting;

    };
}

#endif /*_TOPIC_VECTOR_H_*/