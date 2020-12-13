#include <ros/ros.h>
#include <std_msgs/String.h>
#include <gtk/gtk.h>
#include <boost/thread.hpp>

#include "send_nmea.h"
#include "topic_vector.h"
#include "setting_manager.h"
#include "coodinate_converter.h"
#include "nmeagen/Kenkipos.h"

double count =0;
nmea_convert::TopicVector* gp_topic_vc;
nmea_convert::SendNmea* gp_send_nmea;
struct nmea_convert::OffsetPosition* gp_offset;

const std::string g_ini_path = "/home/jiang/Desktop/bao/openvslam/ros/src/nmeagen_config/NmeaGen.ini";
const std::string g_bias_path = "/home/jiang/Desktop/bao/openvslam/ros/src/nmeagen_config/bias.txt";
const std::string g_geoid_path = "/home/jiang/Desktop/bao/openvslam/ros/src/nmeagen_config/Geoid2000.bin";

void spinThread()
{
  ros::spin();
}

void chatterCallback(const nmeagen::Kenkipos& msg)
{
  ROS_DEBUG("Kenkipos: %f,%f,%f,%d,%d", msg.x, msg.y, msg.z, msg.stamp.sec, msg.stamp.nsec);

  // 適当なメッセージを作成
  // Vectorへ追加
  struct nmea_convert::topic_st item;
  ros::Duration time_offset(gp_offset->sec, gp_offset->nsec);
  item.x = msg.x + gp_offset->x;
  item.y = msg.y + gp_offset->y;
  item.z = msg.z + gp_offset->z;
  item.daytime = msg.stamp + time_offset;

  // 繰り上がり考慮
  if(item.daytime.nsec > 999999999)
  {
    item.daytime.sec++;
    item.daytime.nsec -= 1000000000;
  }

  char date[100];
  time_t t = item.daytime.sec;
  ROS_DEBUG("message_offset: %d[sec] %d[nsec]", item.daytime.sec, item.daytime.nsec);
  strftime(date, sizeof(date), "%Y/%m/%d %a %H:%M:%S", localtime(&t));

  int size = gp_topic_vc->SetItem(item);

  ROS_DEBUG("message_offset: %f,%f,%f,%s.%03u,%d", item.x, item.y, item.z, date, item.daytime.nsec/1000/1000, size);
}

static void cb_st_button(GtkWidget *widget, gpointer data)
{
  gp_send_nmea->SendEnable(true);
}

static void cb_ed_button(GtkWidget *widget, gpointer data)
{
  gp_send_nmea->SendEnable(false);
}


int main(int argc, char** argv)
{
  // 設定読み込み
  nmea_convert::SettingManager* mng = nmea_convert::SettingManager::GetInstance();
  if( !mng->LoadFromFile(g_ini_path) )
  {
    // Loadに失敗した場合、デフォルトを作成
    mng->SaveToFile(g_ini_path);
  }
  gp_offset = &mng->settingData.offset;

  if(mng->LoadBiasFile(g_bias_path))
  {
    // bias.txtの読み込みに成功したら、そちらを使う
    gp_offset = &mng->settingData.bias_offset;
  }

  // ジオイド高ファイル準備
  if( nmea_convert::CoordinateConverter::GetInstance().ReadGeoid2000BinFile2(g_geoid_path) < 0)
  {
    // 読み込み失敗
    ROS_FATAL("Geoid file open error.");
  }

  // ベクター準備
  gp_topic_vc = new nmea_convert::TopicVector(10, mng->settingData.output);

  // ROSノード初期化
  ros::init(argc, argv, "nmeagen_node");
  ros::NodeHandle nh;

  // デバッグ出力がONの場合には、ROS_DEBUGを有効にする
  if(mng->settingData.debug.debug)
  {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
      ros::console::notifyLoggerLevelsChanged();
    }
  }
  nmea_convert::SendNmea send(nh, gp_topic_vc, &mng->settingData);

  send.SendEnable(true);
  gp_send_nmea = &send;

  ros::Subscriber sub = nh.subscribe(mng->settingData.pub.publisherName, 1000, chatterCallback);

  // GUIが使用できるかチェック
  if(gtk_init_check(&argc,&argv))
  {
    // GUI作成
    GtkWidget *window;
    GtkWidget *st_button, *ed_button;
    
    gtk_init(&argc,&argv);
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_widget_set_size_request(window,300,200);

    GtkWidget *box1 = gtk_vbox_new(TRUE,2);
    gtk_container_add (GTK_CONTAINER (window), box1);
    gtk_widget_show(box1);

    st_button = gtk_radio_button_new_with_label(NULL, "開始");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(st_button), TRUE);
    gtk_box_pack_start(GTK_BOX(box1), st_button, TRUE, TRUE, 0);
    gtk_widget_show(st_button);

    GSList *group = gtk_radio_button_get_group(GTK_RADIO_BUTTON(st_button));
    ed_button = gtk_radio_button_new_with_label(group, "停止");
    gtk_box_pack_start(GTK_BOX(box1), ed_button, TRUE, TRUE, 0);
    gtk_widget_show(ed_button);


    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);
    g_signal_connect(st_button, "clicked", G_CALLBACK(cb_st_button), NULL);
    g_signal_connect(ed_button, "clicked", G_CALLBACK(cb_ed_button), NULL);
    gtk_widget_show_all(window);

    boost::thread spin_thread(&spinThread);
    gtk_main();
  }
  else
  {
    // GUI表示ができない場合はNodeのみ起動
    ros::spin();
  }

  // 終了処理
  mng->SaveToFile(g_ini_path);
  delete(gp_topic_vc);

  return 0;
}
