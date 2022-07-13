#include <QMutexLocker>

#include "ROSThread.h"

using namespace std;

ROSThread::ROSThread(QObject *parent, QMutex *th_mutex) :
    QThread(parent), mutex_(th_mutex)
{
  processed_stamp_ = 0;
  play_rate_ = 1.0;
  loop_flag_ = false;
  stop_skip_flag_ = true;
  stereo_active_ = true;
  stereo_thermal_active_ = true;
  stereo_thermal_14bit_left_active_ = true;
  stereo_thermal_14bit_right_active_ = true;

  search_bound_ = 10;
  // search_bound_ = 1000000;
  reset_process_stamp_flag_ = false;
  auto_start_flag_ = true;
  stamp_show_count_ = 0;
  imu_data_version_ = 0;
  prev_clock_stamp_ = 0;
}

ROSThread::~ROSThread()
{
  data_stamp_thread_.active_ = false;
  gps_thread_.active_ = false;
  inspva_thread_.active_ = false;
  imu_thread_.active_ = false;
  ouster_thread_.active_ = false;
  stereo_thread_.active_ = false;
  stereo_thermal_14bit_left_thread_.active_ = false;
  stereo_thermal_14bit_right_thread_.active_ = false;

  usleep(100000);

  data_stamp_thread_.cv_.notify_all();
  if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();

  gps_thread_.cv_.notify_all();
  if(gps_thread_.thread_.joinable()) gps_thread_.thread_.join();
  
  inspva_thread_.cv_.notify_all();
  if(inspva_thread_.thread_.joinable()) inspva_thread_.thread_.join();

  imu_thread_.cv_.notify_all();
  if(imu_thread_.thread_.joinable()) imu_thread_.thread_.join();

  ouster_thread_.cv_.notify_all();
  if(ouster_thread_.thread_.joinable()) ouster_thread_.thread_.join();

  stereo_thread_.cv_.notify_all();
  if(stereo_thread_.thread_.joinable()) stereo_thread_.thread_.join();

  stereo_thermal_14bit_left_thread_.cv_.notify_all();
  if(stereo_thermal_14bit_left_thread_.thread_.joinable()) stereo_thermal_14bit_left_thread_.thread_.join();

  stereo_thermal_14bit_right_thread_.cv_.notify_all();
  if(stereo_thermal_14bit_right_thread_.thread_.joinable()) stereo_thermal_14bit_right_thread_.thread_.join();
}

void ROSThread::ros_initialize(ros::NodeHandle &n)
{
  nh_ = n;

  pre_timer_stamp_ = ros::Time::now().toNSec();
  timer_ = nh_.createTimer(ros::Duration(0.0001), boost::bind(&ROSThread::TimerCallback, this, _1));

  start_sub_  = nh_.subscribe<std_msgs::Bool>("/file_player_start", 1, boost::bind(&ROSThread::FilePlayerStart, this, _1));
  stop_sub_    = nh_.subscribe<std_msgs::Bool>("/file_player_stop", 1, boost::bind(&ROSThread::FilePlayerStop, this, _1));

  gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1000);
  inspva_pub_ = nh_.advertise<novatel_gps_msgs::Inspva>("/inspva", 1000);

  // imu_origin_pub_ = nh_.advertise<irp_sen_msgs::imu>("/imu/data", 1000);
  imu_origin_pub_ = nh_.advertise<irp_sen_msgs::imu>("/xsens_imu_data", 1000);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_raw", 1000);
  magnet_pub_ = nh_.advertise<sensor_msgs::MagneticField>("/imu/mag", 1000);
  ouster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/os_cloud_node/points", 10000);

  stereo_left_pub_ = nh_.advertise<sensor_msgs::Image>("/stereo/left/image_raw", 10);
  stereo_right_pub_ = nh_.advertise<sensor_msgs::Image>("/stereo/right/image_raw", 10);
  stereo_thermal_14bit_left_pub_ = nh_.advertise<sensor_msgs::Image>("/thermal_14bit/left/image_raw", 10);
  stereo_thermal_14bit_right_pub_ = nh_.advertise<sensor_msgs::Image>("/thermal_14bit/right/image_raw", 10);

  stereo_left_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info", 10);
  stereo_right_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/stereo/right/camera_info", 10);
  stereo_thermal_14bit_left_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/thermal_14bit/left/camera_info", 10);
  stereo_thermal_14bit_right_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/thermal_14bit/right/camera_info", 10);

  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
}

void ROSThread::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}

void ROSThread::Ready()
{
  data_stamp_thread_.active_ = false;
  data_stamp_thread_.cv_.notify_all();
  if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();
  gps_thread_.active_ = false;
  gps_thread_.cv_.notify_all();
  if(gps_thread_.thread_.joinable()) gps_thread_.thread_.join();
  inspva_thread_.active_ = false;
  inspva_thread_.cv_.notify_all();
  if(inspva_thread_.thread_.joinable()) inspva_thread_.thread_.join();
  imu_thread_.active_ = false;
  imu_thread_.cv_.notify_all();
  if(imu_thread_.thread_.joinable()) imu_thread_.thread_.join();
  ouster_thread_.active_ = false;
  ouster_thread_.cv_.notify_all();
  if(ouster_thread_.thread_.joinable()) ouster_thread_.thread_.join();
  stereo_thread_.active_ = false;
  stereo_thread_.cv_.notify_all();
  if(stereo_thread_.thread_.joinable()) stereo_thread_.thread_.join();
  stereo_thermal_14bit_left_thread_.active_ = false;
  stereo_thermal_14bit_left_thread_.cv_.notify_all();
  if(stereo_thermal_14bit_left_thread_.thread_.joinable()) stereo_thermal_14bit_left_thread_.thread_.join();
  stereo_thermal_14bit_right_thread_.active_ = false;
  stereo_thermal_14bit_right_thread_.cv_.notify_all();
  if(stereo_thermal_14bit_right_thread_.thread_.joinable()) stereo_thermal_14bit_right_thread_.thread_.join();

  //check path is right or not
  ifstream f((data_folder_path_+"/sensor_data/data_stamp.csv").c_str());
  if(!f.good()){
     cout << "Please check file path. Input path is wrong" << endl;
     return;
  }
  f.close();

  //Read CSV file and make map
  FILE *fp;
  int64_t stamp;
  //data stamp data load

  fp = fopen((data_folder_path_+"/sensor_data/data_stamp.csv").c_str(),"r");
  char data_name[50];
  data_stamp_.clear();
  while(fscanf(fp,"%ld,%s\n",&stamp,data_name) == 2){
//    data_stamp_[stamp] = data_name;
    data_stamp_.insert( multimap<int64_t, string>::value_type(stamp, data_name));
  }
  cout << "Stamp data are loaded" << endl;
  fclose(fp);

  initial_data_stamp_ = data_stamp_.begin()->first - 1;
  last_data_stamp_ = prev(data_stamp_.end(),1)->first - 1;

  //Read inspva data
  fp = fopen((data_folder_path_+"/sensor_data/inspva.csv").c_str(),"r");
  double latitude, longitude, altitude, altitude_orthometric;
  double height, north_velocity, east_velocity, up_velocity, roll, pitch, azimuth;
  // string status;
  char status[17];
  novatel_gps_msgs::Inspva inspva_data;
  inspva_data_.clear();
  while(fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%s",&stamp,&latitude,&longitude,&height,&north_velocity,&east_velocity,&up_velocity,&roll,&pitch,&azimuth,status) == 11){
  //17%19[^\n] %29[^\n]
    inspva_data.header.stamp.fromNSec(stamp);
    inspva_data.header.frame_id = "inspva";
    inspva_data.latitude = latitude;
    inspva_data.longitude = longitude;
    inspva_data.height = height;
    inspva_data.north_velocity = north_velocity;
    inspva_data.east_velocity = east_velocity;
    inspva_data.up_velocity = up_velocity;
    inspva_data.roll = roll;
    inspva_data.pitch = pitch;
    inspva_data.azimuth = azimuth;
    inspva_data.status = status;
    inspva_data_[stamp] = inspva_data;
  }
  cout << "Inspva data are loaded" << endl;
  fclose(fp);

  //Read IMU data
  fp = fopen((data_folder_path_+"/sensor_data/xsens_imu.csv").c_str(),"r");
  double q_x,q_y,q_z,q_w,x,y,z,g_x,g_y,g_z,a_x,a_y,a_z,m_x,m_y,m_z;
  irp_sen_msgs::imu imu_data_origin;
  sensor_msgs::Imu imu_data;
  sensor_msgs::MagneticField mag_data;
  imu_data_.clear();
  mag_data_.clear();

  while(1){
    int length = fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",&stamp,&q_x,&q_y,&q_z,&q_w,&x,&y,&z,&g_x,&g_y,&g_z,&a_x,&a_y,&a_z,&m_x,&m_y,&m_z);
    if(length != 8 && length != 17) break;
    if(length == 8){
      imu_data.header.stamp.fromNSec(stamp);
      imu_data.header.frame_id = "imu";
      imu_data.orientation.x = q_x;
      imu_data.orientation.y = q_y;
      imu_data.orientation.z = q_z;
      imu_data.orientation.w = q_w;

      imu_data_[stamp] = imu_data;
      imu_data_version_ = 1;

      imu_data_origin.header.stamp.fromNSec(stamp);
      imu_data_origin.header.frame_id = "imu";
      imu_data_origin.quaternion_data.x = q_x;
      imu_data_origin.quaternion_data.y = q_y;
      imu_data_origin.quaternion_data.z = q_z;
      imu_data_origin.quaternion_data.w = q_w;
      imu_data_origin.eular_data.x = x;
      imu_data_origin.eular_data.y = y;
      imu_data_origin.eular_data.z = z;
      imu_data_origin_[stamp] = imu_data_origin;

    }else if(length == 17){
      imu_data.header.stamp.fromNSec(stamp);
      imu_data.header.frame_id = "imu";
      imu_data.orientation.x = q_x;
      imu_data.orientation.y = q_y;
      imu_data.orientation.z = q_z;
      imu_data.orientation.w = q_w;
      imu_data.angular_velocity.x = g_x;
      imu_data.angular_velocity.y = g_y;
      imu_data.angular_velocity.z = g_z;
      imu_data.linear_acceleration.x = a_x;
      imu_data.linear_acceleration.y = a_y;
      imu_data.linear_acceleration.z = a_z;

      imu_data.orientation_covariance[0] = 3;
      imu_data.orientation_covariance[4] = 3;
      imu_data.orientation_covariance[8] = 3;
      imu_data.angular_velocity_covariance[0] = 3;
      imu_data.angular_velocity_covariance[4] = 3;
      imu_data.angular_velocity_covariance[8] = 3;
      imu_data.linear_acceleration_covariance[0] = 3;
      imu_data.linear_acceleration_covariance[4] = 3;
      imu_data.linear_acceleration_covariance[8] = 3;


      imu_data_[stamp] = imu_data;
      mag_data.header.stamp.fromNSec(stamp);
      mag_data.header.frame_id = "imu";
      mag_data.magnetic_field.x = m_x;
      mag_data.magnetic_field.y = m_y;
      mag_data.magnetic_field.z = m_z;
      mag_data_[stamp] = mag_data;
      imu_data_version_ = 2;


      imu_data_origin.header.stamp.fromNSec(stamp);
      imu_data_origin.header.frame_id = "imu";
      imu_data_origin.quaternion_data.x = q_x;
      imu_data_origin.quaternion_data.y = q_y;
      imu_data_origin.quaternion_data.z = q_z;
      imu_data_origin.quaternion_data.w = q_w;
      imu_data_origin.eular_data.x = x;
      imu_data_origin.eular_data.y = y;
      imu_data_origin.eular_data.z = z;
      imu_data_origin.gyro_data.x = g_x;
      imu_data_origin.gyro_data.y = g_y;
      imu_data_origin.gyro_data.z = g_z;
      imu_data_origin.acceleration_data.x = a_x;
      imu_data_origin.acceleration_data.y = a_y;
      imu_data_origin.acceleration_data.z = a_z;
      imu_data_origin.magneticfield_data.x = m_x;
      imu_data_origin.magneticfield_data.y = m_y;
      imu_data_origin.magneticfield_data.z = m_z;
      imu_data_origin_[stamp] = imu_data_origin;

    }
  }
  cout << "IMU data are loaded" << endl;
  fclose(fp);

  ouster_file_list_.clear();
  stereo_thermal_14bit_left_file_list_.clear();
  stereo_thermal_14bit_right_file_list_.clear();

  GetDirList(data_folder_path_ + "/sensor_data/ouster",ouster_file_list_);
  GetDirList(data_folder_path_ + "/image/stereo_left",stereo_file_list_);
  GetDirList(data_folder_path_ + "/image/stereo_thermal_14_left",stereo_thermal_14bit_left_file_list_);
  GetDirList(data_folder_path_ + "/image/stereo_thermal_14_right",stereo_thermal_14bit_right_file_list_);

//  GetDirList(data_folder_path_ + "/omni/cam0",omni_file_list_);

  //load camera info

  left_camera_nh_ = ros::NodeHandle(nh_,"left");
  right_camera_nh_ = ros::NodeHandle(nh_,"right");

  left_cinfo_ = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(left_camera_nh_,"/stereo/left"));
  right_cinfo_ = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(right_camera_nh_,"/stereo/right"));


  string left_yaml_file_path = "file://" + data_folder_path_ + "/calibration/left.yaml";
  string right_yaml_file_path = "file://" + data_folder_path_ + "/calibration/right.yaml";


  if(left_cinfo_->validateURL(left_yaml_file_path)){
      left_cinfo_->loadCameraInfo(left_yaml_file_path);
     cout << "Success to load camera info" << endl;
      stereo_left_info_ = left_cinfo_->getCameraInfo();
  }


  if(right_cinfo_->validateURL(right_yaml_file_path)){
      right_cinfo_->loadCameraInfo(right_yaml_file_path);
     cout << "Success to load camera info" << endl;
      stereo_right_info_ = right_cinfo_->getCameraInfo();
  }

//load 14bit thermal camera info

  thermal_14bit_left_camera_nh_ = ros::NodeHandle(nh_,"thermal_14bit_left");
  thermal_14bit_right_camera_nh_ = ros::NodeHandle(nh_,"thermal_14bit_right");

  thermal_14bit_left_cinfo_ = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(left_camera_nh_,"/stereo_thermal_14bit/left"));
  thermal_14bit_right_cinfo_ = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(right_camera_nh_,"/stereo_thermal_14bit/right"));

  string thermal_14bit_left_yaml_file_path = "file://" + data_folder_path_ + "/calibration/thermal_14bit_left.yaml";
  string thermal_14bit_right_yaml_file_path = "file://" + data_folder_path_ + "/calibration/thermal_14bit_right.yaml";

  if(thermal_14bit_left_cinfo_->validateURL(thermal_14bit_left_yaml_file_path)){
      thermal_14bit_left_cinfo_->loadCameraInfo(thermal_14bit_left_yaml_file_path);
      cout << "Success to load thermal camera info" << endl;
      stereo_thermal_14bit_left_info_ = thermal_14bit_left_cinfo_->getCameraInfo();
  }

  if(thermal_14bit_right_cinfo_->validateURL(thermal_14bit_right_yaml_file_path)){
      thermal_14bit_right_cinfo_->loadCameraInfo(thermal_14bit_right_yaml_file_path);
      cout << "Success to load thermal camera info" << endl;
      stereo_thermal_14bit_right_info_ = thermal_14bit_right_cinfo_->getCameraInfo();
  }

  data_stamp_thread_.active_ = true;
  gps_thread_.active_ = true;
  inspva_thread_.active_ = true;
  imu_thread_.active_ = true;
  ouster_thread_.active_ = true;

  stereo_thread_.active_ = true;
  stereo_thermal_14bit_left_thread_.active_ = true;
  stereo_thermal_14bit_right_thread_.active_ = true;

  data_stamp_thread_.thread_ = std::thread(&ROSThread::DataStampThread,this);
  gps_thread_.thread_ = std::thread(&ROSThread::GpsThread,this);
  inspva_thread_.thread_ = std::thread(&ROSThread::InspvaThread,this);
  imu_thread_.thread_ = std::thread(&ROSThread::ImuThread,this);
  ouster_thread_.thread_ = std::thread(&ROSThread::OusterThread,this);
  stereo_thread_.thread_ = std::thread(&ROSThread::StereoThread,this);
  stereo_thermal_14bit_left_thread_.thread_ = std::thread(&ROSThread::StereoThermal14BitLeftThread,this);
  stereo_thermal_14bit_right_thread_.thread_ = std::thread(&ROSThread::StereoThermal14BitRightThread,this);
}

void ROSThread::DataStampThread()
{
  auto stop_region_iter = stop_period_.begin();

  for(auto iter = data_stamp_.begin() ; iter != data_stamp_.end() ; iter ++){
    auto stamp = iter->first;

    while((stamp > (initial_data_stamp_+processed_stamp_))&&(data_stamp_thread_.active_ == true)){
      if(processed_stamp_ == 0){
          iter = data_stamp_.begin();
          stop_region_iter = stop_period_.begin();
          stamp = iter->first;
      }
      usleep(1);
      if(reset_process_stamp_flag_ == true) break;
      //wait for data publish
    }

    if(reset_process_stamp_flag_ == true){
      auto target_stamp = processed_stamp_ + initial_data_stamp_;
      //set iter
      iter = data_stamp_.lower_bound(target_stamp);
      iter = prev(iter,1);
      //set stop region order
      auto new_stamp = iter->first;
      stop_region_iter = stop_period_.upper_bound(new_stamp);

      reset_process_stamp_flag_ = false;
      continue;
    }


    //check whether stop region or not
    if(stamp == stop_region_iter->first){
      if(stop_skip_flag_ == true){
        cout << "Skip stop section!!" << endl;
        iter = data_stamp_.find(stop_region_iter->second);  //find stop region end
        iter = prev(iter,1);
        processed_stamp_ = stop_region_iter->second - initial_data_stamp_;
      }
      stop_region_iter++;
      if(stop_skip_flag_ == true){
        continue;
      }
    }

    if(data_stamp_thread_.active_ == false) return;
    if(iter->second.compare("gps") == 0){
      gps_thread_.push(stamp);
      gps_thread_.cv_.notify_all();
    }else if(iter->second.compare("inspva") == 0){
      inspva_thread_.push(stamp);
      inspva_thread_.cv_.notify_all();
    }else if(iter->second.compare("imu") == 0){
      imu_thread_.push(stamp);
      imu_thread_.cv_.notify_all();
    }else if(iter->second.compare("ouster") == 0){
        ouster_thread_.push(stamp);
        ouster_thread_.cv_.notify_all();
    }else if(iter->second.compare("stereo") == 0 && stereo_active_ == true){
        stereo_thread_.push(stamp);
        stereo_thread_.cv_.notify_all();
    }else if(iter->second.compare("stereo_thermal_14_left") == 0 && stereo_thermal_14bit_left_active_ == true){
        stereo_thermal_14bit_left_thread_.push(stamp);
        stereo_thermal_14bit_left_thread_.cv_.notify_all();
    }else if(iter->second.compare("stereo_thermal_14_right") == 0 && stereo_thermal_14bit_right_active_ == true){
        stereo_thermal_14bit_right_thread_.push(stamp);
        stereo_thermal_14bit_right_thread_.cv_.notify_all();
    }
    stamp_show_count_++;
    if(stamp_show_count_ > 100){
      stamp_show_count_ = 0;
      emit StampShow(stamp);
    }

    if(prev_clock_stamp_ == 0 || (stamp - prev_clock_stamp_) > 10000000){
        rosgraph_msgs::Clock clock;
        clock.clock.fromNSec(stamp);
        clock_pub_.publish(clock);
        prev_clock_stamp_ = stamp;
    }

    if(loop_flag_ == true && iter == prev(data_stamp_.end(),1)){
        iter = data_stamp_.begin();
        stop_region_iter = stop_period_.begin();
        processed_stamp_ = 0;
    }
    if(loop_flag_ == false && iter == prev(data_stamp_.end(),1)){
        play_flag_ = false;
        while(!play_flag_){
            iter = data_stamp_.begin();
            stop_region_iter = stop_period_.begin();
            processed_stamp_ = 0;
            usleep(10000);
        }
    }
  }
  cout << "Data publish complete" << endl;

}

void ROSThread::GpsThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(gps_thread_.mutex_);
    gps_thread_.cv_.wait(ul);
    if(gps_thread_.active_ == false) return;
    ul.unlock();

    while(!gps_thread_.data_queue_.empty()){
      auto data = gps_thread_.pop();
      //process
      if(gps_data_.find(data) != gps_data_.end()){
        gps_pub_.publish(gps_data_[data]);
      }

    }
    if(gps_thread_.active_ == false) return;
  }
}
void ROSThread::InspvaThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(inspva_thread_.mutex_);
    inspva_thread_.cv_.wait(ul);
    if(inspva_thread_.active_ == false) return;
    ul.unlock();

    while(!inspva_thread_.data_queue_.empty()){
      auto data = inspva_thread_.pop();
      //process
      if(inspva_data_.find(data) != inspva_data_.end()){
        inspva_pub_.publish(inspva_data_[data]);
      }

    }
    if(inspva_thread_.active_ == false) return;
  }
}

void ROSThread::ImuThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(imu_thread_.mutex_);
    imu_thread_.cv_.wait(ul);
    if(imu_thread_.active_ == false) return;
    ul.unlock();

    while(!imu_thread_.data_queue_.empty()){
      auto data = imu_thread_.pop();
      //process
      if(imu_data_.find(data) != imu_data_.end()){
        imu_pub_.publish(imu_data_[data]);
        imu_origin_pub_.publish(imu_data_origin_[data]);
        if(imu_data_version_ == 2){
          magnet_pub_.publish(mag_data_[data]);
        }
      }

    }
    if(imu_thread_.active_ == false) return;
  }
}

void ROSThread::TimerCallback(const ros::TimerEvent&)
{
    int64_t current_stamp = ros::Time::now().toNSec();
    if(play_flag_ == true && pause_flag_ == false){
      processed_stamp_ += static_cast<int64_t>(static_cast<double>(current_stamp - pre_timer_stamp_) * play_rate_);
    }
    pre_timer_stamp_ = current_stamp;

    if(play_flag_ == false){
      processed_stamp_ = 0; //reset
      prev_clock_stamp_ = 0;
    }
}

//ouster
void ROSThread::OusterThread()
{
  int current_file_index = 0;
  int previous_file_index = 0;
  while(1){
    std::unique_lock<std::mutex> ul(ouster_thread_.mutex_);
    ouster_thread_.cv_.wait(ul);
    if(ouster_thread_.active_ == false) return;
    ul.unlock();

    while(!ouster_thread_.data_queue_.empty()){
      auto data = ouster_thread_.pop();
      //process

      //publish data
      if(to_string(data) + ".bin" == ouster_next_.first){
        //publish
        ouster_next_.second.header.stamp.fromNSec(data);
        ouster_next_.second.header.frame_id = "ouster";
        ouster_pub_.publish(ouster_next_.second);

      }else{
//        cout << "Re-load right ouster from path" << endl;
        //load current data
        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.clear();
        sensor_msgs::PointCloud2 publish_cloud;
        string current_file_name = data_folder_path_ + "/sensor_data/ouster" +"/"+ to_string(data) + ".bin";
        if(find(next(ouster_file_list_.begin(),max(0,previous_file_index-search_bound_)),ouster_file_list_.end(),to_string(data)+".bin") != ouster_file_list_.end()){
            ifstream file;
            file.open(current_file_name, ios::in|ios::binary);
            while(!file.eof()){
                pcl::PointXYZI point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                cloud.points.push_back (point);
            }
            file.close();

            pcl::toROSMsg(cloud, publish_cloud);
            publish_cloud.header.stamp.fromNSec(data);
            publish_cloud.header.frame_id = "ouster";
            ouster_pub_.publish(publish_cloud);

        }
        previous_file_index = 0;
      }

      //load next data
      pcl::PointCloud<pcl::PointXYZI> cloud;
      cloud.clear();
      sensor_msgs::PointCloud2 publish_cloud;
      current_file_index = find(next(ouster_file_list_.begin(),max(0,previous_file_index-search_bound_)),ouster_file_list_.end(),to_string(data)+".bin") - ouster_file_list_.begin();
      if(find(next(ouster_file_list_.begin(),max(0,previous_file_index-search_bound_)),ouster_file_list_.end(),ouster_file_list_[current_file_index+1]) != ouster_file_list_.end()){
          string next_file_name = data_folder_path_ + "/sensor_data/ouster" +"/"+ ouster_file_list_[current_file_index+1];

          ifstream file;
          file.open(next_file_name, ios::in|ios::binary);
          while(!file.eof()){
              pcl::PointXYZI point;
              file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
              cloud.points.push_back (point);
          }
          file.close();
          pcl::toROSMsg(cloud, publish_cloud);
          ouster_next_ = make_pair(ouster_file_list_[current_file_index+1], publish_cloud);
      }

      previous_file_index = current_file_index;
    }
    if(ouster_thread_.active_ == false) return;
  }
} //end ouster

void ROSThread::StereoThread()
{
  int current_img_index = 0;
  int previous_img_index = 0;

  while(1){
    std::unique_lock<std::mutex> ul(stereo_thread_.mutex_);
    stereo_thread_.cv_.wait(ul);
    if(stereo_thread_.active_ == false) return;
    ul.unlock();

    while(!stereo_thread_.data_queue_.empty()){
      auto data = stereo_thread_.pop();
      //process
      if(stereo_file_list_.size() == 0) continue;

      //publish
      if(to_string(data)+".png" == stereo_left_next_img_.first && !stereo_left_next_img_.second.empty() && !stereo_right_next_img_.second.empty()){
        cv_bridge::CvImage left_out_msg;
        left_out_msg.header.stamp.fromNSec(data);
        left_out_msg.header.frame_id = "stereo_left";
        left_out_msg.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
        left_out_msg.image    = stereo_left_next_img_.second;

        cv_bridge::CvImage right_out_msg;
        right_out_msg.header.stamp.fromNSec(data);
        right_out_msg.header.frame_id = "stereo_right";
        right_out_msg.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
        right_out_msg.image    = stereo_right_next_img_.second;

        stereo_left_info_.header.stamp.fromNSec(data);
        stereo_left_info_.header.frame_id = "/stereo/left";
        stereo_right_info_.header.stamp.fromNSec(data);
        stereo_right_info_.header.frame_id = "/stereo/right";

        stereo_left_pub_.publish(left_out_msg.toImageMsg());
        stereo_right_pub_.publish(right_out_msg.toImageMsg());

        stereo_left_info_pub_.publish(stereo_left_info_);
        stereo_right_info_pub_.publish(stereo_right_info_);

      }else{
//        cout << "Re-load stereo image from image path" << endl;

        string current_stereo_left_name = data_folder_path_ + "/image/stereo_left" +"/"+ to_string(data)+".png";
        string current_stereo_right_name = data_folder_path_ + "/image/stereo_right" +"/"+ to_string(data)+".png";
        cv::Mat current_left_image;
        cv::Mat current_right_image;
        current_left_image = imread(current_stereo_left_name, CV_LOAD_IMAGE_ANYDEPTH);
        current_right_image = imread(current_stereo_right_name, CV_LOAD_IMAGE_ANYDEPTH);

        if(!current_left_image.empty() && !current_right_image.empty()){

            cv_bridge::CvImage left_out_msg;
            left_out_msg.header.stamp.fromNSec(data);
            left_out_msg.header.frame_id = "stereo_left";
            left_out_msg.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
            left_out_msg.image    = current_left_image;

            cv_bridge::CvImage right_out_msg;
            right_out_msg.header.stamp.fromNSec(data);
            right_out_msg.header.frame_id = "stereo_right";
            right_out_msg.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
            right_out_msg.image    = current_right_image;

            stereo_left_info_.header.stamp.fromNSec(data);
            stereo_left_info_.header.frame_id = "/stereo/left";
            stereo_right_info_.header.stamp.fromNSec(data);
            stereo_right_info_.header.frame_id = "/stereo/right";

            stereo_left_pub_.publish(left_out_msg.toImageMsg());
            stereo_right_pub_.publish(right_out_msg.toImageMsg());

            stereo_left_info_pub_.publish(stereo_left_info_);
            stereo_right_info_pub_.publish(stereo_right_info_);
        }
        previous_img_index = 0;


      }

      //load next image
      current_img_index = find(next(stereo_file_list_.begin(), max(0,previous_img_index - search_bound_)),stereo_file_list_.end(),to_string(data)+".png") - stereo_file_list_.begin();
      if(current_img_index < stereo_file_list_.size()-2){

          string next_stereo_left_name = data_folder_path_ + "/image/stereo_left" +"/"+ stereo_file_list_[current_img_index+1];
          string next_stereo_right_name = data_folder_path_ + "/image/stereo_right" +"/"+ stereo_file_list_[current_img_index+1];
          cv::Mat next_left_image;
          cv::Mat next_right_image;
          next_left_image = imread(next_stereo_left_name, CV_LOAD_IMAGE_ANYDEPTH);
          next_right_image = imread(next_stereo_right_name, CV_LOAD_IMAGE_ANYDEPTH);
          if(!next_left_image.empty() && !next_right_image.empty()){
              stereo_left_next_img_ = make_pair(stereo_file_list_[current_img_index+1], next_left_image);
              stereo_right_next_img_ = make_pair(stereo_file_list_[current_img_index+1], next_right_image);
          }

      }
      previous_img_index = current_img_index;
    }
    if(stereo_thread_.active_ == false) return;
  }
}

//14bit thermal Left Thread
void ROSThread::StereoThermal14BitLeftThread()
{
  int current_img_index = 0;
  int previous_img_index = 0;

  while(1){
    std::unique_lock<std::mutex> ul(stereo_thermal_14bit_left_thread_.mutex_);
    stereo_thermal_14bit_left_thread_.cv_.wait(ul);
    if(stereo_thermal_14bit_left_thread_.active_ == false) return;
    ul.unlock();

    while(!stereo_thermal_14bit_left_thread_.data_queue_.empty()){
      auto data = stereo_thermal_14bit_left_thread_.pop();
      //process
      if(stereo_thermal_14bit_left_file_list_.size() == 0) continue;

      //publish
      if(to_string(data)+".png" == stereo_thermal_14bit_left_next_img_.first && !stereo_thermal_14bit_left_next_img_.second.empty()){
        cv_bridge::CvImage left_out_msg;
        left_out_msg.header.stamp.fromNSec(data);
        left_out_msg.header.frame_id = "stereo_thermal_14bit_left";
        left_out_msg.encoding = sensor_msgs::image_encodings::MONO16;
        left_out_msg.image    = stereo_thermal_14bit_left_next_img_.second;
       
        stereo_thermal_14bit_left_info_.header.stamp.fromNSec(data);
        stereo_thermal_14bit_left_info_.header.frame_id = "/stereo_thermal_14bit/left";



        stereo_thermal_14bit_left_pub_.publish(left_out_msg.toImageMsg());
        stereo_thermal_14bit_left_info_pub_.publish(stereo_thermal_14bit_left_info_);

      }else{
//        cout << "Re-load stereo image from image path" << endl;

        string current_stereo_thermal_14bit_left_name = data_folder_path_ + "/image/stereo_thermal_14_left" +"/"+ to_string(data)+".png";
        cv::Mat current_left_image;
        current_left_image = imread(current_stereo_thermal_14bit_left_name, CV_LOAD_IMAGE_ANYDEPTH);
  
        if(!current_left_image.empty()){

            cv_bridge::CvImage left_out_msg;
            left_out_msg.header.stamp.fromNSec(data);
            left_out_msg.header.frame_id = "stereo_thermal_14bit_left";
            left_out_msg.encoding = sensor_msgs::image_encodings::MONO16;
            left_out_msg.image    = current_left_image;

            stereo_thermal_14bit_left_info_.header.stamp.fromNSec(data);
            stereo_thermal_14bit_left_info_.header.frame_id = "/stereo_thermal_14bit/left";
            stereo_thermal_14bit_left_pub_.publish(left_out_msg.toImageMsg());
            stereo_thermal_14bit_left_info_pub_.publish(stereo_thermal_14bit_left_info_);
        }
        previous_img_index = 0;
      }

      //load next image
      current_img_index = find(next(stereo_thermal_14bit_left_file_list_.begin(), max(0,previous_img_index - search_bound_)),stereo_thermal_14bit_left_file_list_.end(),to_string(data)+".png") - stereo_thermal_14bit_left_file_list_.begin();
      if(current_img_index < stereo_file_list_.size()-2){
          string next_stereo_thermal_14bit_left_name = data_folder_path_ + "/image/stereo_thermal_14_left" +"/"+ stereo_thermal_14bit_left_file_list_[current_img_index+1];
          cv::Mat next_left_image;
          next_left_image = imread(next_stereo_thermal_14bit_left_name, CV_LOAD_IMAGE_ANYDEPTH);
          if(!next_left_image.empty()){
              stereo_thermal_14bit_left_next_img_ = make_pair(stereo_thermal_14bit_left_file_list_[current_img_index+1], next_left_image);
          }
      }
      previous_img_index = current_img_index;
    }
    if(stereo_thermal_14bit_left_thread_.active_ == false) return;
  }
}
//end thermal Left Thread

//14bit thermal Right Thread
void ROSThread::StereoThermal14BitRightThread()
{
  int current_img_index = 0;
  int previous_img_index = 0;

  while(1){
    std::unique_lock<std::mutex> ul(stereo_thermal_14bit_right_thread_.mutex_);
    stereo_thermal_14bit_right_thread_.cv_.wait(ul);
    if(stereo_thermal_14bit_right_thread_.active_ == false) return;
    ul.unlock();

    while(!stereo_thermal_14bit_right_thread_.data_queue_.empty()){
      auto data = stereo_thermal_14bit_right_thread_.pop();
      //process
      if(stereo_thermal_14bit_right_file_list_.size() == 0) continue;

      //publish
      if(to_string(data)+".png" == stereo_thermal_14bit_right_next_img_.first && !stereo_thermal_14bit_right_next_img_.second.empty()){
        cv_bridge::CvImage right_out_msg;
        right_out_msg.header.stamp.fromNSec(data);
        right_out_msg.header.frame_id = "stereo_thermal_14bit_right";
        right_out_msg.encoding = sensor_msgs::image_encodings::MONO16;
        right_out_msg.image    = stereo_thermal_14bit_right_next_img_.second;
       
        stereo_thermal_14bit_right_info_.header.stamp.fromNSec(data);
        stereo_thermal_14bit_right_info_.header.frame_id = "/stereo_thermal_14bit/right";

        stereo_thermal_14bit_right_pub_.publish(right_out_msg.toImageMsg());
        stereo_thermal_14bit_right_info_pub_.publish(stereo_thermal_14bit_right_info_);

      }else{
//        cout << "Re-load stereo image from image path" << endl;

        string current_stereo_thermal_14bit_right_name = data_folder_path_ + "/image/stereo_thermal_14_right" +"/"+ to_string(data)+".png";
        cv::Mat current_right_image;
        current_right_image = imread(current_stereo_thermal_14bit_right_name, CV_LOAD_IMAGE_ANYDEPTH);

        if(!current_right_image.empty()){

            cv_bridge::CvImage right_out_msg;
            right_out_msg.header.stamp.fromNSec(data);
            right_out_msg.header.frame_id = "stereo_thermal_14bit_right";
            right_out_msg.encoding = sensor_msgs::image_encodings::MONO16;
            right_out_msg.image    = current_right_image;

            stereo_thermal_14bit_right_info_.header.stamp.fromNSec(data);
            stereo_thermal_14bit_right_info_.header.frame_id = "/stereo_thermal_14bit/right";
            stereo_thermal_14bit_right_pub_.publish(right_out_msg.toImageMsg());
            stereo_thermal_14bit_right_info_pub_.publish(stereo_thermal_14bit_right_info_);
        }
        previous_img_index = 0;
      }

      //load next image
      current_img_index = find(next(stereo_thermal_14bit_right_file_list_.begin(), max(0,previous_img_index - search_bound_)),stereo_thermal_14bit_right_file_list_.end(),to_string(data)+".png") - stereo_thermal_14bit_right_file_list_.begin();
      if(current_img_index < stereo_file_list_.size()-2){
          string next_stereo_thermal_14bit_right_name = data_folder_path_ + "/image/stereo_thermal_14_right" +"/"+ stereo_thermal_14bit_right_file_list_[current_img_index+1];
          cv::Mat next_right_image;
          next_right_image = imread(next_stereo_thermal_14bit_right_name, CV_LOAD_IMAGE_ANYDEPTH);
          if(!next_right_image.empty()){
              stereo_thermal_14bit_right_next_img_ = make_pair(stereo_thermal_14bit_right_file_list_[current_img_index+1], next_right_image);
          }
      }
      previous_img_index = current_img_index;
    }
    if(stereo_thermal_14bit_right_thread_.active_ == false) return;
  }
}
//end thermal Right Thread

int ROSThread::GetDirList(string dir, vector<string> &files)
{

  vector<string> tmp_files;
  struct dirent **namelist;
  int n;
  n = scandir(dir.c_str(),&namelist, 0 , alphasort);
  if (n < 0)
      perror("scandir");
  else {
      while (n--) {
      if(string(namelist[n]->d_name) != "." && string(namelist[n]->d_name) != ".."){
        tmp_files.push_back(string(namelist[n]->d_name));
      }
      free(namelist[n]);
      }
      free(namelist);
  }

  for(auto iter = tmp_files.rbegin() ; iter!= tmp_files.rend() ; iter++){
    files.push_back(*iter);
  }
    return 0;
}

void ROSThread::FilePlayerStart(const std_msgs::BoolConstPtr& msg)
{
  if(auto_start_flag_ == true){
    cout << "File player auto start" << endl;
    usleep(1000000);
    play_flag_ = false;
    emit StartSignal();
  }
}

void ROSThread::FilePlayerStop(const std_msgs::BoolConstPtr& msg)
{
  cout << "File player auto stop" << endl;
  play_flag_ = true;
  emit StartSignal();
}
void ROSThread::ResetProcessStamp(int position)
{
  if(position > 0 && position < 10000){
    processed_stamp_ = static_cast<int64_t>(static_cast<float>(last_data_stamp_ - initial_data_stamp_)*static_cast<float>(position)/static_cast<float>(10000));
    reset_process_stamp_flag_ = true;
  }
}
