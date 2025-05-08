#include "stereo-inertial-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(SLAM)
{
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;

    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    if (doRectify_)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }

    subImu_ = this->create_subscription<ImuMsg>("imu0", 1000, std::bind(&StereoInertialNode::GrabImu, this, _1));
    subImgLeft_ = this->create_subscription<ImageMsg>("cam0/image_raw", 100, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    subImgRight_ = this->create_subscription<ImageMsg>("cam1/image_raw", 100, std::bind(&StereoInertialNode::GrabImageRight, this, _1));

    // ROS2 publishers
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("body_odom", 10);
    tracked_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("tracked_points", 10);
    all_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("all_points", 10);
    kf_markers_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("kf_markers", 10);

    // TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);


    syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu, this);
}

StereoInertialNode::~StereoInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void StereoInertialNode::GrabImageLeft(const ImageMsg::SharedPtr msgLeft)
{
    bufMutexLeft_.lock();

    if (!imgLeftBuf_.empty())
        imgLeftBuf_.pop();
    imgLeftBuf_.push(msgLeft);

    bufMutexLeft_.unlock();
}

void StereoInertialNode::GrabImageRight(const ImageMsg::SharedPtr msgRight)
{
    bufMutexRight_.lock();

    if (!imgRightBuf_.empty())
        imgRightBuf_.pop();
    imgRightBuf_.push(msgRight);

    bufMutexRight_.unlock();
}

cv::Mat StereoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

sensor_msgs::msg::PointCloud2 StereoInertialNode::ConvertMapPointsToPointCloud2(
    const std::vector<ORB_SLAM3::MapPoint*>& map_points,
    rclcpp::Time stamp)
{
    sensor_msgs::msg::PointCloud2 msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.height = 1;
    msg.width = map_points.size();
    msg.is_dense = false;
    msg.is_bigendian = false;

    msg.fields.resize(3);
    std::string field_names[3] = {"x", "y", "z"};

    for (size_t i = 0; i < 3; ++i)
    {
        msg.fields[i].name = field_names[i];
        msg.fields[i].offset = i * sizeof(float);
        msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[i].count = 1;
    }

    msg.point_step = 3 * sizeof(float);
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(msg.row_step);

    unsigned char* ptr = msg.data.data();
    for (auto* pt : map_points)
    {
        if (pt)
        {
            Eigen::Vector3f p = pt->GetWorldPos();
            float data[3] = {p.x(), p.y(), p.z()};
            std::memcpy(ptr, data, sizeof(data));
            ptr += msg.point_step;
        }
    }

    return msg;
}



void StereoInertialNode::PublishOutputs(double timestamp_sec)
{
    rclcpp::Time msg_time(timestamp_sec * 1e9);  // seconds to nanoseconds
    // auto Tcw = SLAM_->GetCamTwc();

    // if (Tcw.translation().array().isNaN()[0]) return;

    // geometry_msgs::msg::PoseStamped pose_msg;
    // pose_msg.header.frame_id = "map";
    // pose_msg.header.stamp = msg_time;
    // pose_msg.pose.position.x = Tcw.translation().x();
    // pose_msg.pose.position.y = Tcw.translation().y();
    // pose_msg.pose.position.z = Tcw.translation().z();
    // auto q = Tcw.unit_quaternion();
    // pose_msg.pose.orientation.x = q.x();
    // pose_msg.pose.orientation.y = q.y();
    // pose_msg.pose.orientation.z = q.z();
    // pose_msg.pose.orientation.w = q.w();
    // pose_pub_->publish(pose_msg);

    // geometry_msgs::msg::TransformStamped tf_msg;
    // tf_msg.header.stamp = msg_time;
    // tf_msg.header.frame_id = "map";
    // tf_msg.child_frame_id = "camera";
    // tf_msg.transform.translation.x = Tcw.translation().x();
    // tf_msg.transform.translation.y = Tcw.translation().y();
    // tf_msg.transform.translation.z = Tcw.translation().z();
    // tf_msg.transform.rotation.x = q.x();
    // tf_msg.transform.rotation.y = q.y();
    // tf_msg.transform.rotation.z = q.z();
    // tf_msg.transform.rotation.w = q.w();
    // tf_broadcaster_->sendTransform(tf_msg);

    tracked_points_pub_->publish(ConvertMapPointsToPointCloud2(SLAM_->GetTrackedMapPoints(), msg_time));
    // all_points_pub_->publish(ConvertMapPointsToPointCloud2(SLAM_->GetAllMapPoints(), msg_time));
}




void StereoInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while (1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf_.empty() && !imgRightBuf_.empty() && !imuBuf_.empty())
        {
            tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);

            bufMutexRight_.lock();
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1)
            {
                imgRightBuf_.pop();
                tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
            }
            bufMutexRight_.unlock();

            bufMutexLeft_.lock();
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1)
            {
                imgLeftBuf_.pop();
                tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            }
            bufMutexLeft_.unlock();

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
            {
                std::cout << "big time difference" << std::endl;
                continue;
            }
            if (tImLeft > Utility::StampToSec(imuBuf_.back()->header.stamp))
                continue;

            bufMutexLeft_.lock();
            imLeft = GetImage(imgLeftBuf_.front());
            imgLeftBuf_.pop();
            bufMutexLeft_.unlock();

            bufMutexRight_.lock();
            imRight = GetImage(imgRightBuf_.front());
            imgRightBuf_.pop();
            bufMutexRight_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImLeft)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            if (bClahe_)
            {
                clahe_->apply(imLeft, imLeft);
                clahe_->apply(imRight, imRight);
            }

            if (doRectify_)
            {
                cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
                cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
            }

            SLAM_->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);

            // Publish outputs
            PublishOutputs(tImLeft);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}
