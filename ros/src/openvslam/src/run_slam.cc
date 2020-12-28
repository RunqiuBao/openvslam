#ifdef USE_PANGOLIN_VIEWER
#include <pangolin_viewer/viewer.h>
#elif USE_SOCKET_PUBLISHER
#include <socket_publisher/publisher.h>
#endif

#include <openvslam/system.h>
#include <openvslam/config.h>

#include <iostream>
#include <chrono>
#include <numeric>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include <spdlog/spdlog.h>
#include <popl.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <openvslam/Kenkipos.h>

#include "openvslam/util/stereo_rectifier.h"
#include "openvslam/util/image_converter.h"

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

int temp = 0;
// bool isinitial=true;
// double initialZ=0;

class Listener
{
public:
    std::chrono::time_point<std::chrono::steady_clock> tp_0;
    std::vector<double> *track_times;
    openvslam::system *SLAM;
    ros::Publisher *kenkipos_publisher;
    openvslam::util::stereo_rectifier *rectifier;
    image_transport::Publisher *publisher;
    Listener(std::chrono::time_point<std::chrono::steady_clock> tp_0, std::vector<double> *track_times, openvslam::system *SLAM, ros::Publisher *kenkipos_publisher, openvslam::util::stereo_rectifier *rectifier)
    {
        this->tp_0=tp_0;
        this->track_times=track_times;
        this->SLAM=SLAM;
        this->kenkipos_publisher=kenkipos_publisher;
        this->rectifier=rectifier;

    }
    void SLAMcallback(const sensor_msgs::ImageConstPtr& leftimage, const sensor_msgs::ImageConstPtr& rightimage)//, const sensor_msgs::ImageConstPtr& maskimage)
    {
      // Solve all of perception here...
      const auto tp_1 = std::chrono::steady_clock::now();
      const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0).count();
      const auto leftimagetimestamp = leftimage->header.stamp;

      // input the current frame and estimate the camera pose
      //Eigen::Matrix4d campose;
      cv::Mat imgl;
      cv::Mat imgr;
      cv::Mat imgmask;
      try{
        imgl = cv_bridge::toCvShare(leftimage, "rgb8")->image;
        imgr = cv_bridge::toCvShare(rightimage, "rgb8")->image;
        // imgmask = cv_bridge::toCvShare(maskimage, "mono8")->image;
      }
      catch(cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
      }

      //rectify
      cv::Mat left_img_rect, right_img_rect;
      const bool equal_hist = true; 
      if (equal_hist) {
          openvslam::util::equalize_histogram(imgl);
          openvslam::util::equalize_histogram(imgr);
      }
      (*rectifier).rectify(imgl, imgr, left_img_rect, right_img_rect);
    //   std::cout<< "Width : " << imgl.size().width << std::endl;
    //   std::cout<< "Height: " << imgl.size().height << std::endl;
    //   cv::imwrite("/home/jiang/Desktop/bao/openvslam/rectify-result/l/"+std::to_string(temp)+".jpg", left_img_rect);
    //   cv::imwrite("/home/jiang/Desktop/bao/openvslam/rectify-result/r/"+std::to_string(temp)+".jpg", right_img_rect);

      auto cam_pose_cw = (*SLAM).feed_stereo_frame(left_img_rect, right_img_rect, timestamp, imgmask);
    //   const auto campose = (*SLAM).feed_monocular_frame(imgl, timestamp, imgmask);
      const openvslam::Mat44_t cam_pose_wc=cam_pose_cw.inverse();//Twc
      const openvslam::Vec3_t& trans_wc = cam_pose_wc.block<3, 1>(0, 3);
      std::cout<<cam_pose_wc<<std::endl;
      double x=trans_wc[0];
      double y=trans_wc[1];
      double z=trans_wc[2];

    //   const auto td::chrono::steady_clock::now();

    //   const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
    //   (*track_times).push_back(track_time);

      // coordinates transform
      cv::Mat Twc0 = (cv::Mat_<double>(4,4)<<
      -0.99984545,  0.01384197, -0.0108385 ,  0.68684061,
      0.01651846,  0.52864174, -0.84868431,  1.99247474,
      -0.00601777, -0.84873218, -0.52878869,  2.97021366,
      0.        ,  0.        ,  0.        ,  1.        );//right hand
      cv::Mat Pc_homo = (cv::Mat_<double>(4,1)<<x,y,z,1);
      cv::Mat Pw_homo = Twc0*Pc_homo;
      std::cout<<std::to_string(Pw_homo.rows)<<","<<std::to_string(Pw_homo.cols)<<std::endl;
      std::cout<<Pw_homo<<std::endl;
      x = Pw_homo.at<double>(0,0);
      y = Pw_homo.at<double>(1,0);
      z = Pw_homo.at<double>(2,0);
      

      // world coordinate broadcast
      openvslam::Kenkipos msg;
      msg.stamp = leftimagetimestamp;
    //   ros::Duration timeoffset = ros::Duration(32400);
    //   msg.stamp -= timeoffset; 
    //   x = x + 22621.588;
    //   y = y + 8681.259;
    //   z = z + 27.295835;
    //   x=-x;
    //   y=-y;
    //   if(isinitial){
    //     initialZ = -z;
    //     isinitial = false;
    //   }
    //   z=z+2*initialZ;
    //   x+=22621.588;
    //   y+=8681.259;
    //   z+=27.295835;
      msg.x = y;
      msg.y = x;
      msg.z = z;
      (*kenkipos_publisher).publish(msg);
    }
};

void stereo_tracking(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path,
                   const std::string& mask_img_path, const bool eval_log, const std::string& map_db_path){
    // load the mask image
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);

    openvslam::util::stereo_rectifier rectifier(cfg);

    // build a SLAM system
    openvslam::system SLAM(cfg, vocab_file_path);
    // startup the SLAM process
    SLAM.startup();

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif

    std::vector<double> track_times;
    const auto tp_0 = std::chrono::steady_clock::now();

    // initialize this node
    ros::NodeHandle nh;

    // publisher for kenkipos broadcast
    ros::Publisher kenkipos_publisher = nh.advertise<openvslam::Kenkipos>("Kisekipos", 1000);

    Listener listener(tp_0, &track_times, &SLAM, &kenkipos_publisher, &rectifier);

    message_filters::Subscriber<sensor_msgs::Image> leftimage_sub(nh, "imagel", 1);
    message_filters::Subscriber<sensor_msgs::Image> rightimage_sub(nh, "imager", 1);
    // message_filters::Subscriber<sensor_msgs::Image> maskimage_sub(nh, "maskimage", 1);
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), leftimage_sub, rightimage_sub);//, maskimage_sub);
    std::cout << "callback registered!" << std::endl;
    sync.registerCallback(boost::bind(&Listener::SLAMcallback, &listener, _1, _2));//, _3));//_1, _2, _3 represents the three synchronized subscriber objects.

    //****test****//
    // ros::Publisher kenkipos_publisher2 = nh.advertise<openvslam::Kenkipos>("Kisekipos2", 1000);
    // cv::Mat Twc0 = (cv::Mat_<double>(4,4)<<
    //     1, 0, 0, 0,
    //     0, 1, 0, 0,
    //     0, 0, 1, 0,
    //     0, 0, 0, 1);
    // cv::Mat Pc_homo = (cv::Mat_<double>(4,1)<<0,0,0,1);
    // cv::Mat Pw_homo = Twc0.mul(Pc_homo);
    // std::cout<<std::to_string(Pw_homo.rows)<<","<<std::to_string(Pw_homo.cols)<<std::endl;
    // double x;
    // double y;
    // double z;
    // x = Pw_homo.at<double>(0,0);
    // y = Pw_homo.at<double>(1,0);
    // z = Pw_homo.at<double>(2,0);
    // openvslam::Kenkipos msg;
    // msg.stamp = ros::Time::now();
    // msg.x = x;
    // msg.y = y;
    // msg.z = z;
    // ros::Rate loop_rate(10);
    // while (ros::ok()){
    //     msg.stamp = ros::Time::now();
    //     (kenkipos_publisher2).publish(msg);
    //     // ROS_INFO("published");
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
     //****test****//
    

    // run the viewer in another thread
#ifdef USE_PANGOLIN_VIEWER
    std::thread thread([&]() {
        viewer.run();
        if (SLAM.terminate_is_requested()) {
            // wait until the loop BA is finished
            while (SLAM.loop_BA_is_running()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
            ros::shutdown();
        }
    });
#elif USE_SOCKET_PUBLISHER
    std::thread thread([&]() {
        publisher.run();
        if (SLAM.terminate_is_requested()) {
            // wait until the loop BA is finished
            while (SLAM.loop_BA_is_running()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
            ros::shutdown();
        }
    });
#endif

    ros::spin();

    // automatically close the viewer
#ifdef USE_PANGOLIN_VIEWER
    viewer.request_terminate();
    thread.join();
#elif USE_SOCKET_PUBLISHER
    publisher.request_terminate();
    thread.join();
#endif

    // shutdown the SLAM process
    SLAM.shutdown();

    if (eval_log) {
        // output the trajectories for evaluation
        SLAM.save_frame_trajectory("frame_trajectory.txt", "TUM");
        SLAM.save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
        // output the tracking times for evaluation
        std::ofstream ofs("track_times.txt", std::ios::out);
        if (ofs.is_open()) {
            for (const auto track_time : track_times) {
                ofs << track_time << std::endl;
            }
            ofs.close();
        }
    }

    if (!map_db_path.empty()) {
        // output the map database
        SLAM.save_map_database(map_db_path);
    }

    if (track_times.size()) {
        std::sort(track_times.begin(), track_times.end());
        const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
        std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
        std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
    }


}

void mono_tracking(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path,
                   const std::string& mask_img_path, const bool eval_log, const std::string& map_db_path) {
    // load the mask image
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);

    // build a SLAM system
    openvslam::system SLAM(cfg, vocab_file_path);
    // startup the SLAM process
    SLAM.startup();

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif

    std::vector<double> track_times;
    const auto tp_0 = std::chrono::steady_clock::now();

    // initialize this node
    const ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // run the SLAM as subscriber
    image_transport::Subscriber sub = it.subscribe("imagel", 1, [&](const sensor_msgs::ImageConstPtr& msg) {
        const auto tp_1 = std::chrono::steady_clock::now();
        const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0).count();

        // input the current frame and estimate the camera pose
        SLAM.feed_monocular_frame(cv_bridge::toCvShare(msg, "bgr8")->image, timestamp, mask);

        const auto tp_2 = std::chrono::steady_clock::now();

        const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
        track_times.push_back(track_time);
    });

    // run the viewer in another thread
#ifdef USE_PANGOLIN_VIEWER
    std::thread thread([&]() {
        viewer.run();
        if (SLAM.terminate_is_requested()) {
            // wait until the loop BA is finished
            while (SLAM.loop_BA_is_running()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
            ros::shutdown();
        }
    });
#elif USE_SOCKET_PUBLISHER
    std::thread thread([&]() {
        publisher.run();
        if (SLAM.terminate_is_requested()) {
            // wait until the loop BA is finished
            while (SLAM.loop_BA_is_running()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
            ros::shutdown();
        }
    });
#endif

    ros::spin();

    // automatically close the viewer
#ifdef USE_PANGOLIN_VIEWER
    viewer.request_terminate();
    thread.join();
#elif USE_SOCKET_PUBLISHER
    publisher.request_terminate();
    thread.join();
#endif

    // shutdown the SLAM process
    SLAM.shutdown();

    if (eval_log) {
        // output the trajectories for evaluation
        SLAM.save_frame_trajectory("frame_trajectory.txt", "TUM");
        SLAM.save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
        // output the tracking times for evaluation
        std::ofstream ofs("track_times.txt", std::ios::out);
        if (ofs.is_open()) {
            for (const auto track_time : track_times) {
                ofs << track_time << std::endl;
            }
            ofs.close();
        }
    }

    if (!map_db_path.empty()) {
        // output the map database
        SLAM.save_map_database(map_db_path);
    }

    if (track_times.size()) {
        std::sort(track_times.begin(), track_times.end());
        const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
        std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
        std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
    }
}

int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
#endif
    ros::init(argc, argv, "run_slam");

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto setting_file_path = op.add<popl::Value<std::string>>("c", "config", "setting file path");
    auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
    auto eval_log = op.add<popl::Switch>("", "eval-log", "store trajectory and tracking times for evaluation");
    auto map_db_path = op.add<popl::Value<std::string>>("", "map-db", "store a map database at this path after SLAM", "");
    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!vocab_file_path->is_set() || !setting_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>(setting_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif

    // run tracking
    if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {
        mono_tracking(cfg, vocab_file_path->value(), mask_img_path->value(), eval_log->is_set(), map_db_path->value());
    }
    else {
        stereo_tracking(cfg, vocab_file_path->value(), mask_img_path->value(), eval_log->is_set(), map_db_path->value());
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}
