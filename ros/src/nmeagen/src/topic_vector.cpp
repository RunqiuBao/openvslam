/**
 * @file topic_vector.cpp
 * @brief ROSから受信したtopicをvectorへ貯めておくクラス
 * @author ISL
 * 
 */

#include "topic_vector.h"

namespace nmea_convert
{
    TopicVector::TopicVector(int size, nmea_convert::OutputSetting& output)
    {
        _size = size;
        _vc.clear();
        _p_outputSetting = &output;
    }

    /**
     * @fn
     * struct topic_stを受け取り、vector(LILO)へPushする関数
     * @brief 要約説明
     * @param (const struct topic_st* item) vectorに追加するアイテム
     * @return 現在のアイテム数
     */
    int TopicVector::SetItem(const struct topic_st& item)
    {
        std::lock_guard<std::recursive_mutex> lock(_mutex);

        if(_vc.size() >= 1)
        {
            auto last = (_vc.back());

            // 距離チェック
            double dx = item.x - last.x;
            double dy = item.y - last.y;
            double dz = item.z - last.z;

            double dist = std::sqrt(dx*dx+dy*dy+dz*dz);
            if(dist >= _p_outputSetting->posErrThreshold)
            {
                ROS_INFO("Distance over %f >= %f lastpos(%f,%f,%f)-pos(%f,%f,%f)", dist, _p_outputSetting->posErrThreshold, last.x, last.y, last.z, item.x, item.y, item.z);
                return -1;
            }
        }
        _vc.push_back(item);

        if(_vc.size() > _size)  _vc.erase(_vc.begin());

        return _vc.size();
    }

    /**
     * @fn
     * vectorからアイテムを一つ取り出す。
     * vectorにアイテムが無い場合には、falseを返却する
     * @brief 要約説明
     * @param (struct topic_st& item) vectorから取得したアイテム
     * @return アイテムが取得できたか
     */
    bool TopicVector::GetItem(struct topic_st& output_item)
    {
        if(_vc.empty()) return false;

        std::lock_guard<std::recursive_mutex> lock(_mutex);

        // 末尾を取得
        output_item = _vc.back();

    char date[100];
    time_t t = output_item.daytime.sec;
    strftime(date, sizeof(date), "%Y/%m/%d %a %H:%M:%S", localtime(&t));
    ROS_DEBUG("GetItem:%s(%u.%04u)", date, output_item.daytime.sec, output_item.daytime.nsec/1000/1000);

        return true;
    }

    /**
     * @fn
     * 過去の点から速度を推定し、現在時間までプロパゲートする
     * 過去の点が足りていない場合には、直近の値をそのまま出力する
     * @brief 要約説明
     * @param (struct topic_st& output_item) 指定時間までプロパゲートした位置情報
     * @return 出力アイテムが存在するか
     */
    bool TopicVector::GetPropergateItem(struct topic_st& output_item, int num_pos, ros::Time now)
    {
        // データが空の場合には計算できない
        if(_vc.empty()) return false;

        std::lock_guard<std::recursive_mutex> lock(_mutex);

        // 
        struct topic_st last_pos = *(_vc.end()-1);

        // num_pos以下の場合には計算不能なため、直近の値をそのまま出力
        if(_vc.size() < num_pos || num_pos < 2)
        {
            output_item = last_pos;
            return true;
        }
        // debug
        //now = last_pos.daytime;

        // _vc.size() >= num_pos が確定
        // 速度推定 過去位置の差分で計算 とりあえず一つ前の位置から速度推定
        double x_vel[num_pos-1];
        double y_vel[num_pos-1];
        double z_vel[num_pos-1];

        auto target_pos = _vc.cend();
        target_pos--;
        for(int i=0; i<(num_pos-1); ++i )
        {
            // 一つ手前の位置を取得
            auto target_pos_bf = target_pos;
            target_pos_bf--;
            ROS_DEBUG("Speed[%d]: pos1=%f,%f,%f", i, target_pos->x, target_pos->y, target_pos->z);
            ROS_DEBUG("Speed[%d]: pos2=%f,%f,%f", i, target_pos_bf->x, target_pos_bf->y, target_pos_bf->z);

            ros::Duration duration = target_pos->daytime - target_pos_bf->daytime;
            ROS_DEBUG("TimeDiff: %u.%09u-%u.%09u=%f", target_pos->daytime.sec, target_pos->daytime.nsec, target_pos_bf->daytime.sec, target_pos_bf->daytime.nsec, (duration.sec + duration.nsec/1000.0/1000.0/1000.0));
            // 位置差分をm/s へ変換
            x_vel[i] = (target_pos->x - target_pos_bf->x) / (duration.sec + duration.nsec/1000.0/1000.0/1000.0);
            y_vel[i] = (target_pos->y - target_pos_bf->y) / (duration.sec + duration.nsec/1000.0/1000.0/1000.0);
            z_vel[i] = (target_pos->z - target_pos_bf->z) / (duration.sec + duration.nsec/1000.0/1000.0/1000.0);

            // 次の位置へ
            target_pos = target_pos_bf;
        }

        // 速度平均（単純に平均）
        double x_val_total=0,y_val_total=0,z_val_total=0;
        int vel_size = sizeof(x_vel)/sizeof(double);
        for(int i=0; i<vel_size; i++)
        {
            ROS_DEBUG("Speed[%d]: %f,%f,%f", i, x_vel[i], y_vel[i], z_vel[i]);
            x_val_total += x_vel[i];
            y_val_total += y_vel[i];
            z_val_total += z_vel[i];
        }
        x_val_total = x_val_total / vel_size;
        y_val_total = y_val_total / vel_size;
        z_val_total = z_val_total / vel_size;


        ros::Duration duration_last = now - last_pos.daytime;
        ROS_DEBUG("LastTimeDiff: %u.%09u-%u.%09u=%f", now.sec, now.nsec, last_pos.daytime.sec, last_pos.daytime.nsec, (duration_last.sec + duration_last.nsec/1000.0/1000.0/1000.0));

        double diff_duration = duration_last.sec + duration_last.nsec / 1000.0 / 1000.0 / 1000.0;

        output_item.x = last_pos.x + x_val_total * diff_duration;
        output_item.y = last_pos.y + y_val_total * diff_duration;
        output_item.z = last_pos.z + z_val_total * diff_duration;
        ROS_DEBUG("Last: (%f,%f,%f) >>> (%f,%f,%f))", last_pos.x, last_pos.y, last_pos.z, output_item.x, output_item.y, output_item.z);

        output_item.daytime = now;

        return true;
    }

    /**
     * @fn
     * 過去の点から2D速度を計算する
     * 過去の点が足りていない場合には、何もしない
     * @brief 要約説明
     * @param (struct topic_st& output_item) 指定時間までプロパゲートした位置情報
     * @return 出力アイテムが存在するか
     */
    bool TopicVector::GetSpeedKph(double& output_speed, int num_pos)
    {
        // データが空の場合には計算できない
        if(_vc.empty()) return false;

        std::lock_guard<std::recursive_mutex> lock(_mutex);

        // num_pos以下の場合には計算不能なため、直近の値をそのまま出力
        if(_vc.size() < num_pos || num_pos < 2)
        {
            return false;
        }

        struct topic_st last_pos = *(_vc.end()-1);

        // _vc.size() >= num_pos が確定
        // 速度推定 過去位置の差分で計算 とりあえず一つ前の位置から速度推定
        double x_vel[num_pos-1];
        double y_vel[num_pos-1];
        double z_vel[num_pos-1];

        auto target_pos = _vc.cend();
        target_pos--;
        for(int i=0; i<(num_pos-1); ++i )
        {
            // 一つ手前の位置を取得
            auto target_pos_bf = target_pos;
            target_pos_bf--;
            ROS_DEBUG("Speed[%d]: pos1=%f,%f,%f", i, target_pos->x, target_pos->y, target_pos->z);
            ROS_DEBUG("Speed[%d]: pos2=%f,%f,%f", i, target_pos_bf->x, target_pos_bf->y, target_pos_bf->z);

            ros::Duration duration = target_pos->daytime - target_pos_bf->daytime;
            ROS_DEBUG("TimeDiff: %u.%09u-%u.%09u=%f", target_pos->daytime.sec, target_pos->daytime.nsec, target_pos_bf->daytime.sec, target_pos_bf->daytime.nsec, (duration.sec + duration.nsec/1000.0/1000.0/1000.0));
            // 位置差分をm/s へ変換
            x_vel[i] = (target_pos->x - target_pos_bf->x) / (duration.sec + duration.nsec/1000.0/1000.0/1000.0);
            y_vel[i] = (target_pos->y - target_pos_bf->y) / (duration.sec + duration.nsec/1000.0/1000.0/1000.0);
            z_vel[i] = (target_pos->z - target_pos_bf->z) / (duration.sec + duration.nsec/1000.0/1000.0/1000.0);

            // 次の位置へ
            target_pos = target_pos_bf;
        }

        // 速度平均（単純に平均）
        double x_val_total=0,y_val_total=0,z_val_total=0;
        int vel_size = sizeof(x_vel)/sizeof(double);
        for(int i=0; i<vel_size; i++)
        {
            ROS_DEBUG("Speed[%d]: %f,%f,%f", i, x_vel[i], y_vel[i], z_vel[i]);
            x_val_total += x_vel[i];
            y_val_total += y_vel[i];
            z_val_total += z_vel[i];
        }
        x_val_total = x_val_total / vel_size;
        y_val_total = y_val_total / vel_size;
        z_val_total = z_val_total / vel_size;

        // 2D速度計算 [m/s]=>[km/h]
        double speed = std::sqrt(x_val_total*x_val_total+y_val_total*y_val_total);
        speed = speed * 60.0 * 60.0 / 1000.0;

        output_speed = speed;

        return true;
    }


}

