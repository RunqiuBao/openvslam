#include "openvslam/camera/perspective.h"
#include "openvslam/camera/fisheye.h"
#include "openvslam/util/stereo_rectifier.h"

#include <spdlog/spdlog.h>
#include <opencv2/imgproc.hpp>

namespace openvslam {
namespace util {

stereo_rectifier::stereo_rectifier(const std::shared_ptr<openvslam::config>& cfg)
    : stereo_rectifier(cfg->camera_, cfg->yaml_node_) {}

stereo_rectifier::stereo_rectifier(camera::base* camera, const YAML::Node& yaml_node)
    : model_type_(load_model_type(yaml_node)) {
    spdlog::debug("CONSTRUCT: util::stereo_rectifier");
    assert(camera->setup_type_ == camera::setup_type_t::Stereo);
    // set image size
    const cv::Size img_size(camera->cols_, camera->rows_);
    // set camera matrices  //runqiu: modify rectifier
    ////const auto K_l = parse_vector_as_mat(cv::Size(3, 3), yaml_node["StereoRectifier.K_left"].as<std::vector<double>>());
    ////const auto K_r = parse_vector_as_mat(cv::Size(3, 3), yaml_node["StereoRectifier.K_right"].as<std::vector<double>>());
    std::vector<double> kleft{yaml_node["Camera.fx"].as<double>(), 0, yaml_node["Camera.cx"].as<double>(), 0, yaml_node["Camera.fy"].as<double>(), yaml_node["Camera.cy"].as<double>(), 0, 0, 1};
    std::vector<double> kright{yaml_node["Camera.fx_right"].as<double>(), 0, yaml_node["Camera.cx_right"].as<double>(), 0, yaml_node["Camera.fy_right"].as<double>(), yaml_node["Camera.cy_right"].as<double>(), 0, 0, 1};
    const auto K_l = parse_vector_as_mat(cv::Size(3, 3), kleft);
    const auto K_r = parse_vector_as_mat(cv::Size(3, 3), kright);
    // set rotation matrices
    ////const auto R_l = parse_vector_as_mat(cv::Size(3, 3), yaml_node["StereoRectifier.R_left"].as<std::vector<double>>());
    ////const auto R_r = parse_vector_as_mat(cv::Size(3, 3), yaml_node["StereoRectifier.R_right"].as<std::vector<double>>());
    std::vector<double> r_vec=yaml_node["Camera.rvec_left_to_right"].as<std::vector<double>>();
    cv::Mat Rright = cv::Mat::eye(3, 3, CV_32F);
    cv::Rodrigues(r_vec, Rright);
    const auto R_l = cv::Mat::eye(3, 3, CV_32F);
    const auto R_r = Rright;
    // set distortion parameters depending on the camera model
    ////const auto D_l_vec = yaml_node["StereoRectifier.D_left"].as<std::vector<double>>();
    ////const auto D_r_vec = yaml_node["StereoRectifier.D_right"].as<std::vector<double>>();
    std::vector<double> Dleft={yaml_node["Camera.k1"].as<double>(), yaml_node["Camera.k2"].as<double>(), yaml_node["Camera.k3"].as<double>(), yaml_node["Camera.k4"].as<double>()};
    std::vector<double> Dright={yaml_node["Camera.k1_right"].as<double>(), yaml_node["Camera.k2_right"].as<double>(), yaml_node["Camera.k3_right"].as<double>(), yaml_node["Camera.k4_right"].as<double>()};
    const auto D_l_vec = Dleft;
    const auto D_r_vec = Dright;

    switch (model_type_) {
        case camera::model_type_t::Perspective: {
            const auto D_l = parse_vector_as_mat(cv::Size(1, D_l_vec.size()), D_l_vec);
            const auto D_r = parse_vector_as_mat(cv::Size(1, D_r_vec.size()), D_r_vec);
            // create undistortion maps
            auto c = static_cast<camera::perspective*>(camera);
            cv::initUndistortRectifyMap(K_l, D_l, R_l, c->cv_cam_matrix_, img_size, CV_32F, undist_map_x_l_, undist_map_y_l_);
            cv::initUndistortRectifyMap(K_r, D_r, R_r, c->cv_cam_matrix_, img_size, CV_32F, undist_map_x_r_, undist_map_y_r_);
            break;
        }
        case camera::model_type_t::Fisheye: {
            const auto D_l = parse_vector_as_mat(cv::Size(1, D_l_vec.size()), D_l_vec);
            const auto D_r = parse_vector_as_mat(cv::Size(1, D_r_vec.size()), D_r_vec);
            // create undistortion maps
            // camera model is perspective after rectification
            auto c = static_cast<camera::perspective*>(camera);
            cv::fisheye::initUndistortRectifyMap(K_l, D_l, R_l, c->cv_cam_matrix_, img_size, CV_32F, undist_map_x_l_, undist_map_y_l_);
            cv::fisheye::initUndistortRectifyMap(K_r, D_r, R_r, c->cv_cam_matrix_, img_size, CV_32F, undist_map_x_r_, undist_map_y_r_);
            break;
        }
        default: {
            throw std::runtime_error("Invalid model type for stereo rectification: " + camera->get_model_type_string());
        }
    }
}

stereo_rectifier::~stereo_rectifier() {
    spdlog::debug("DESTRUCT: util::stereo_rectifier");
}

void stereo_rectifier::rectify(const cv::Mat& in_img_l, const cv::Mat& in_img_r,
                               cv::Mat& out_img_l, cv::Mat& out_img_r) const {
    cv::remap(in_img_l, out_img_l, undist_map_x_l_, undist_map_y_l_, cv::INTER_LINEAR);
    cv::remap(in_img_r, out_img_r, undist_map_x_r_, undist_map_y_r_, cv::INTER_LINEAR);
}

cv::Mat stereo_rectifier::parse_vector_as_mat(const cv::Size& shape, const std::vector<double>& vec) {
    cv::Mat mat(shape, CV_64F);
    std::memcpy(mat.data, vec.data(), shape.height * shape.width * sizeof(double));
    return mat;
}

camera::model_type_t stereo_rectifier::load_model_type(const YAML::Node& yaml_node) {
    ////const auto model_type_str = yaml_node["StereoRectifier.model"].as<std::string>("perspective");
    const auto model_type_str = yaml_node["Camera.model"].as<std::string>("fisheye");;//runqiu: temporary
    if (model_type_str == "perspective") {
        return camera::model_type_t::Perspective;
    }
    else if (model_type_str == "fisheye") {
        return camera::model_type_t::Fisheye;
    }
    else if (model_type_str == "equirectangular") {
        return camera::model_type_t::Equirectangular;
    }

    throw std::runtime_error("Invalid camera model: " + model_type_str);
}

} // namespace util
} // namespace openvslam
