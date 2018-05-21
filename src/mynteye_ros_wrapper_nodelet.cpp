#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>

#include <image_transport/image_transport.h>

#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>

#include "camera.h"
#include "utility.h"
#include "calibration_parameters.h"
#include "init_parameters.h"

// #define VERBOSE
// #define VERBOSE_TO_FILE

namespace mynt_wrapper {

#ifdef USE_OPENCV2
void CompatDistCoeffs(cv::Mat &distCoeffs) {
    int w = distCoeffs.cols;
    if (w >= 8) {
        w = 8;
    } else if (w >= 5) {
        w = 5;
    } else if (w >= 4) {
        w = 4;
    } else {
        CV_Assert(false);
    }
    distCoeffs = distCoeffs.row(0).colRange(0,w);
}
#endif

class MYNTWrapperNodelet : public nodelet::Nodelet {

    ros::NodeHandle nh;
    ros::NodeHandle nh_ns;
    boost::shared_ptr<boost::thread> device_poll_thread;

    mynteye::Camera cam;
    mynteye::Resolution resolution;

    image_transport::Publisher pub_raw_right;
    image_transport::Publisher pub_raw_left;
    image_transport::Publisher pub_depth;
    image_transport::Publisher pub_left;
    image_transport::Publisher pub_right;

    ros::Publisher pub_left_cam_info;
    ros::Publisher pub_right_cam_info;
    ros::Publisher pub_imu;

    std::string right_frame_id;
    std::string left_frame_id;
    std::string depth_frame_id;
    std::string imu_frame_id;

    int device_name;

    bool enable_cpu;
    bool enable_depth;

#ifdef VERBOSE_TO_FILE
    std::ofstream file_imus;
#endif

sensor_msgs::ImagePtr imageToROSmsg(const cv::Mat &img,
        const std::string &encodingType,
        const std::string &frameId,
        const ros::Time &stamp) {
    sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
    sensor_msgs::Image& imgMessage = *ptr;
    imgMessage.header.stamp = stamp;
    imgMessage.header.frame_id = frameId;
    imgMessage.height = img.rows;
    imgMessage.width = img.cols;
    imgMessage.encoding = encodingType;
    int num = 1;
    imgMessage.is_bigendian = !(*(char *) &num == 1);
    imgMessage.step = img.cols * img.elemSize();
    size_t size = imgMessage.step * img.rows;
    imgMessage.data.resize(size);

    if (img.isContinuous())
        memcpy((char*) (&imgMessage.data[0]), img.data, size);
    else {
        uchar* opencvData = img.data;
        uchar* rosData = (uchar*) (&imgMessage.data[0]);
        for (unsigned int i = 0; i < img.rows; i++) {
            memcpy(rosData, opencvData, imgMessage.step);
            rosData += imgMessage.step;
            opencvData += img.step;
        }
    }

    return ptr;
}

void publishImage(const cv::Mat &img,
        const image_transport::Publisher &pub_img,
        const std::string &img_frame_id,
        const ros::Time &stamp) {
    pub_img.publish(imageToROSmsg(img, sensor_msgs::image_encodings::MONO8, img_frame_id, stamp));
}

void publishDepth(cv::Mat &depth, const ros::Time &stamp) {
    depth.convertTo(depth, CV_16UC1);
    pub_depth.publish(imageToROSmsg(depth, sensor_msgs::image_encodings::TYPE_16UC1, depth_frame_id, stamp));
}

void publishIMU(const mynteye::IMUData &imudata, const ros::Time &stamp) {
    sensor_msgs::Imu msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = imu_frame_id;

    msg.linear_acceleration.x = imudata.accel_x * 9.8;
    msg.linear_acceleration.y = imudata.accel_y * 9.8;
    msg.linear_acceleration.z = imudata.accel_z * 9.8;

    msg.linear_acceleration_covariance[0] = 0.04;
    msg.linear_acceleration_covariance[1] = 0;
    msg.linear_acceleration_covariance[2] = 0;

    msg.linear_acceleration_covariance[3] = 0;
    msg.linear_acceleration_covariance[4] = 0.04;
    msg.linear_acceleration_covariance[5] = 0;

    msg.linear_acceleration_covariance[6] = 0;
    msg.linear_acceleration_covariance[7] = 0;
    msg.linear_acceleration_covariance[8] = 0.04;

    msg.angular_velocity.x = imudata.gyro_x / 57.2956;
    msg.angular_velocity.y = imudata.gyro_y / 57.2956;
    msg.angular_velocity.z = imudata.gyro_z / 57.2956;

    msg.angular_velocity_covariance[0] = 0.02;
    msg.angular_velocity_covariance[1] = 0;
    msg.angular_velocity_covariance[2] = 0;

    msg.angular_velocity_covariance[3] = 0;
    msg.angular_velocity_covariance[4] = 0.02;
    msg.angular_velocity_covariance[5] = 0;

    msg.angular_velocity_covariance[6] = 0;
    msg.angular_velocity_covariance[7] = 0;
    msg.angular_velocity_covariance[8] = 0.02;

    pub_imu.publish(msg);
}

void publishCamInfo(const sensor_msgs::CameraInfoPtr &cam_info_msg,
        const ros::Publisher &pub_cam_info,
        const ros::Time &stamp) {
    static int seq = 0;
    cam_info_msg->header.stamp = stamp;
    cam_info_msg->header.seq = seq;
    pub_cam_info.publish(cam_info_msg);
    ++seq;
}

void fillCamInfo(const mynteye::Resolution &resolution,
        mynteye::CalibrationParameters* calibration_parameters,
        const sensor_msgs::CameraInfoPtr &left_cam_info_msg,
        const sensor_msgs::CameraInfoPtr &right_cam_info_msg,
        const std::string &left_frame_id,
        const std::string &right_frame_id) {
    int width = resolution.width;
    int height = resolution.height;

    left_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    right_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    left_cam_info_msg->D.resize(5);
    right_cam_info_msg->D.resize(5);

    left_cam_info_msg->D[0] = calibration_parameters->D1.at<double>(0, 0);
    right_cam_info_msg->D[0] = calibration_parameters->D2.at<double>(0, 0);

    left_cam_info_msg->D[1] = calibration_parameters->D1.at<double>(0, 1);
    right_cam_info_msg->D[1] = calibration_parameters->D2.at<double>(0, 1);

    left_cam_info_msg->D[2] = calibration_parameters->D1.at<double>(0, 7);
    right_cam_info_msg->D[2] = calibration_parameters->D2.at<double>(0, 7);

    left_cam_info_msg->D[3] = 0.0;
    right_cam_info_msg->D[3] = 0.0;

    left_cam_info_msg->D[4] = 0.0;
    right_cam_info_msg->D[4] = 0.0;

    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            left_cam_info_msg->K[i * 3 + j] = calibration_parameters->M1.at<double>(i, j);
            right_cam_info_msg->K[i * 3 + j] = calibration_parameters->M2.at<double>(i, j);

            left_cam_info_msg->R[i * 3 + j] = calibration_parameters->R.at<double>(i, j);
            right_cam_info_msg->R[i * 3 + j] = calibration_parameters->R.at<double>(i, j);
        }
    }

    cv::Size img_size(width, height);
    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect leftROI, rightROI;
#ifdef USE_OPENCV2
    CompatDistCoeffs(calibration_parameters->D1);
    CompatDistCoeffs(calibration_parameters->D2);
#endif
    cv::stereoRectify(calibration_parameters->M1, calibration_parameters->D1,
            calibration_parameters->M2, calibration_parameters->D2, img_size,
            calibration_parameters->R, calibration_parameters->T, R1, R2, P1, P2, Q,
            cv::CALIB_ZERO_DISPARITY, 0, img_size, &leftROI, &rightROI);

    for(int i = 0; i < R1.rows; i++) {
        for(int j = 0; j < R1.cols; j++) {
            left_cam_info_msg->R[i * R1.cols + j] = R1.at<double>(i, j);
            right_cam_info_msg->R[i * R1.cols + j] = R2.at<double>(i, j);
        }
    }
    for(int i = 0; i < P1.rows; i++) {
        for(int j = 0; j < P1.cols; j++) {
            left_cam_info_msg->P[i * P1.cols + j] = P1.at<double>(i, j);
            right_cam_info_msg->P[i * P1.cols + j] = P2.at<double>(i, j);
        }
    }
    left_cam_info_msg->width = right_cam_info_msg->width = width;
    left_cam_info_msg->height = right_cam_info_msg->height = height;
    left_cam_info_msg->header.frame_id = left_frame_id;
    right_cam_info_msg->header.frame_id = right_frame_id;
}

ros::Time getImgStamp(bool reset = false) {
    static std::uint32_t img_time_beg = -1;
    static double img_ros_time_beg;
    if (reset) {
        img_time_beg = -1;
        return ros::Time::now();
    }
    if (img_time_beg == -1) {
        img_time_beg = cam.GetTimestamp();
        img_ros_time_beg = ros::Time::now().toSec();
    }
    return ros::Time(img_ros_time_beg + (cam.GetTimestamp() - img_time_beg) * 0.0001f);
}

ros::Time getIMUStamp(mynteye::IMUData *imudata, bool reset = false) {
    static std::uint32_t imu_time_beg = -1;
    static double imu_ros_time_beg;
    if (reset) {
        imu_time_beg = -1;
        return ros::Time::now();
    }
    if (imu_time_beg == -1) {
        imu_time_beg = imudata->time;
        imu_ros_time_beg = ros::Time::now().toSec();
    }
    return ros::Time(imu_ros_time_beg + (imudata->time - imu_time_beg) * 0.0001f);
}

void device_poll() {
    using namespace mynteye;

    if (enable_cpu) {
        // If you wanna run with CPU instead of GPU
        cam.SetMode(Mode::MODE_CPU);
    } else {
        // otherwise, auto detect
    }

    resolution = cam.GetResolution();

    InitParameters params(std::to_string(device_name));
    cam.Open(params);

    if (!cam.IsOpened()) {
        std::cerr << "Error: Open camera failed" << std::endl;
        return;
    }

    if (enable_depth) {
        cam.ActivateDepthMapFeature();
    }

    CalibrationParameters *calib_params = nullptr;
    calib_params = new CalibrationParameters(cam.GetCalibrationParameters());
    cv::Mat img_left, img_right,depthmap,leftImRGB,rightImRGB;

    sensor_msgs::CameraInfoPtr left_cam_info_msg(new sensor_msgs::CameraInfo());
    sensor_msgs::CameraInfoPtr right_cam_info_msg(new sensor_msgs::CameraInfo());
    //sensor_msgs::CameraInfoPtr depth_cam_info_msg(new sensor_msgs::CameraInfo());
    fillCamInfo(resolution, calib_params, left_cam_info_msg, right_cam_info_msg, left_frame_id, right_frame_id);

    std::vector<IMUData> imudatas;
    std::uint32_t timestamp = 0;

#ifdef VERBOSE
    std::uint64_t img_count = 0;
    std::uint64_t imu_count = 0;
    //double loop_beg = ros::Time::now().toSec();

#ifdef VERBOSE_TO_FILE
    std::uint64_t imu_get_count = 0;
    double imu_get_beg;
#endif
#endif

    while (nh_ns.ok()) {

        if (cam.Grab() != ErrorCode::SUCCESS) continue;

        int left_SubNumber = pub_left.getNumSubscribers();
        int left_raw_SubNumber = pub_raw_left.getNumSubscribers();
        int right_SubNumber = pub_right.getNumSubscribers();
        int right_raw_SubNumber = pub_raw_right.getNumSubscribers();
        int depth_SubNumber = pub_depth.getNumSubscribers();
        int imu_SubNumber = pub_imu.getNumSubscribers();
        bool runLoop = (left_SubNumber + left_raw_SubNumber + right_SubNumber + right_raw_SubNumber + depth_SubNumber + imu_SubNumber) > 0;
        if (runLoop) {
            bool img_get = false;
            if (left_SubNumber > 0 &&
                    cam.RetrieveImage(leftImRGB, View::VIEW_LEFT) == ErrorCode::SUCCESS) {
                publishCamInfo(left_cam_info_msg, pub_left_cam_info, getImgStamp());
                publishImage(leftImRGB, pub_left, left_frame_id, getImgStamp());
                img_get = true;
            }
            if (left_raw_SubNumber > 0 &&
                    cam.RetrieveImage(img_left, View::VIEW_LEFT_UNRECTIFIED) == ErrorCode::SUCCESS) {
                publishCamInfo(left_cam_info_msg, pub_left_cam_info, getImgStamp());
                publishImage(img_left, pub_raw_left, left_frame_id, getImgStamp());
                img_get = true;
            }
            if (right_SubNumber > 0 &&
                    cam.RetrieveImage(rightImRGB, View::VIEW_RIGHT) == ErrorCode::SUCCESS) {
                publishCamInfo(right_cam_info_msg, pub_right_cam_info, getImgStamp());
                publishImage(rightImRGB, pub_right, right_frame_id, getImgStamp());
                img_get = true;
            }
            if (right_raw_SubNumber > 0 &&
                    cam.RetrieveImage(img_right, View::VIEW_RIGHT_UNRECTIFIED) == ErrorCode::SUCCESS) {
                publishCamInfo(right_cam_info_msg, pub_right_cam_info, getImgStamp());
                publishImage(img_right, pub_raw_right, right_frame_id, getImgStamp());
                img_get = true;
            }
            if (enable_depth && depth_SubNumber > 0) {
                if (cam.RetrieveImage(depthmap, View::VIEW_DEPTH_MAP) == ErrorCode::SUCCESS) {
                    publishDepth(depthmap, getImgStamp());
                }
            }
            if (!img_get) {
                getImgStamp(true);  // reset
            }

#ifdef VERBOSE
            if (img_get) {
                ++img_count;
                std::cout << "Img count: " << img_count << std::endl;
                std::cout << "    time: " << cam.GetTimestamp()
                          << ", stamp: " << getImgStamp() << std::endl
                          << std::endl;
            }
#endif

            if (imu_SubNumber > 0) {
                if (cam.RetrieveIMUData(imudatas, timestamp) == ErrorCode::SUCCESS && !imudatas.empty()) {
                    size_t size = imudatas.size();

#ifdef VERBOSE
                    imu_count += size;
                    std::cout << "IMU count: " << imu_count << std::endl;
#ifdef VERBOSE_TO_FILE
                    if (imu_get_count == 0) {
                        imu_get_beg = ros::Time::now().toSec();
                        // ~/.ros/imus_publish.txt
                        file_imus = std::ofstream("imus_publish.txt", std::ios::out);
                    }
                    file_imus << (++imu_get_count) << ", " << size << " IMUs, " << imu_count << " in total." << std::endl;
#endif
#endif

                    for (size_t i = 0, n = size-1; i <= n; ++i) {
                        auto &imudata = imudatas[i];
                        ros::Time imu_ros_time = getIMUStamp(&imudata);

#ifdef VERBOSE
                        std::cout << "  IMU[" << i << "]"
                            << " accel(" << imudata.accel_x << "," << imudata.accel_y << "," << imudata.accel_z << ")"
                            << ", gyro(" << imudata.gyro_x << "," << imudata.gyro_y << "," << imudata.gyro_z << ")"
                            << std::endl;
                        std::cout << "    time: " << imudata.time
                            << ", stamp: " << imu_ros_time
                            << ", time_offset: " << imudata.time_offset << std::endl;
#ifdef VERBOSE_TO_FILE
                        file_imus << "IMU[" << i << "] stamp: " << std::fixed << imu_ros_time
                            << ", accel(" << imudata.accel_x << "," << imudata.accel_y << "," << imudata.accel_z << ")"
                            << ", gyro(" << imudata.gyro_x << "," << imudata.gyro_y << "," << imudata.gyro_z << ")"
                            << std::endl;
#endif
#endif

                        if (imu_SubNumber > 0) publishIMU(imudata, imu_ros_time);
                        // Sleep 1ms, otherwise publish may drop some IMUs.
                        ros::Duration(0.001).sleep();
                    }
#ifdef VERBOSE
                    std::cout << std::endl;
#ifdef VERBOSE_TO_FILE
                    file_imus << std::endl;
#endif
#endif
                } else {
                    getIMUStamp(nullptr, true);  // reset
                }
            } else {
                getIMUStamp(nullptr, true);  // reset
            }

            // Sleep to control camera hz.
            //loop_rate.sleep();
        }

#ifdef VERBOSE
#ifdef VERBOSE_TO_FILE
        if (imu_SubNumber <= 0 && imu_get_count > 0) {
            double imu_get_end = ros::Time::now().toSec();
            double elapsed = imu_get_end - imu_get_beg;
            file_imus << "time beg: " << std::fixed << imu_get_beg << " s" << std::endl
                << "time end: " << std::fixed << imu_get_end << " s" << std::endl
                << "time cost: " << elapsed << " s" << std::endl << std::endl
                << "imu: " << (imu_count / imu_get_count) << " per frame, "
                    << imu_count << " in total, " << (imu_count / elapsed) << " Hz" << std::endl;

            imu_get_count = 0;
            imu_count = 0;
        }
#endif
#endif
    }
}

void onInit() {
    std::string img_topic = "image_rect_color";
    std::string img_raw_topic = "image_raw_color";

    std::string left_topic = "left/" + img_topic;
    std::string left_raw_topic = "left/" + img_raw_topic;
    std::string left_cam_info_topic = "left/camera_info";
    left_frame_id = "/mynt_left_frame";

    std::string right_topic = "right/" + img_topic;
    std::string right_raw_topic = "right/" + img_raw_topic;
    std::string right_cam_info_topic = "right/camera_info";
    right_frame_id = "/mynt_right_frame";

    std::string depth_topic = "depth/depth_registered";
    depth_frame_id = "/mynt_depth_frame";

    std::string imu_topic = "imu";
    imu_frame_id = "/mynt_imu_frame";

    device_name = 1;
    enable_cpu = false;
    enable_depth = false;

    nh = getMTNodeHandle();
    nh_ns = getMTPrivateNodeHandle();
    nh_ns.getParam("device_name", device_name);
    nh_ns.getParam("enable_cpu", enable_cpu);
    nh_ns.getParam("enable_depth", enable_depth);
    nh_ns.getParam("left_topic", left_topic);
    nh_ns.getParam("left_raw_topic", left_raw_topic);
    nh_ns.getParam("left_cam_info_topic", left_cam_info_topic);
    nh_ns.getParam("right_topic", right_topic);
    nh_ns.getParam("right_raw_topic", right_raw_topic);
    nh_ns.getParam("right_cam_info_topic", right_cam_info_topic);
    nh_ns.getParam("depth_topic", depth_topic);
    nh_ns.getParam("imu_topic", imu_topic);

    image_transport::ImageTransport it_mynteye(nh);

    pub_left = it_mynteye.advertise(left_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << left_topic);
    pub_raw_left = it_mynteye.advertise(left_raw_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << left_raw_topic);
    pub_left_cam_info = nh.advertise<sensor_msgs::CameraInfo>(left_cam_info_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << left_cam_info_topic);

    pub_right = it_mynteye.advertise(right_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << right_topic);
    pub_raw_right = it_mynteye.advertise(right_raw_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << right_raw_topic);
    pub_right_cam_info = nh.advertise<sensor_msgs::CameraInfo>(right_cam_info_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << right_cam_info_topic);

    pub_depth = it_mynteye.advertise(depth_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << depth_topic);

    pub_imu = nh.advertise<sensor_msgs::Imu>(imu_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << imu_topic);


    device_poll_thread = boost::shared_ptr<boost::thread>
            (new boost::thread(boost::bind(&MYNTWrapperNodelet::device_poll, this)));
}

~MYNTWrapperNodelet() {
    cam.Close();
}

};

}  // namespace mynt_wrapper

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mynt_wrapper::MYNTWrapperNodelet, nodelet::Nodelet);
