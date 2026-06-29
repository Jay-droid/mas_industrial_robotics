/*
 * Copyright 2019 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil
 * ROS2 contributors: Hamsa Datta Perur, Shubham Shinde, Vamsi Kalagaturu, Vivek
 * Mannava.
 *
 */

#include "mir_object_recognition/multimodal_object_recognition.hpp"

namespace perception_namespace {
MultiModalObjectRecognitionROS::MultiModalObjectRecognitionROS(
  const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("mmor_node", options)

{
  scene_segmentation_ros_ =
    SceneSegmentationROSSPtr(new SceneSegmentationROS());
  mm_object_recognition_utils_ = MultimodalObjectRecognitionUtilsSPtr(
    new MultimodalObjectRecognitionUtils());

  MultiModalObjectRecognitionROS::declare_all_parameters();
}

void
MultiModalObjectRecognitionROS::synchronizeCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr& image,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud)
{
  // save the latest data only if data is not locked
  if (!data_lock_.owns_lock()) {
    this->image_msg_ = image;
    this->pointcloud_msg_ = cloud;
  }
}

void
MultiModalObjectRecognitionROS::recognizedCloudCallback(
  const mir_interfaces::msg::ObjectList& msg)
{
  RCLCPP_INFO(get_logger(), "Received recognized cloud callback");
  if (!received_recognized_cloud_list_flag_) {
    recognized_cloud_list_ = msg;
    received_recognized_cloud_list_flag_ = true;
  }
}

  void
  MultiModalObjectRecognitionROS::recognizedImageCallback(
      const mir_interfaces::msg::ObjectList &msg)
  {
    if (!waiting_for_recognized_image_.load())
    {
      return;
    }

    std::lock_guard<std::mutex> lock(recognized_image_sync_mutex_);
    recognized_image_list_ = msg;

    if (recognized_image_promise_)
    {
      try
      {
        recognized_image_promise_->set_value();
      }
      catch (const std::future_error &)
      {
        // Promise may already be fulfilled by a previous callback.
      }
    }
  }

  bool
  MultiModalObjectRecognitionROS::requestRemoteRgbRecognition(
      const sensor_msgs::msg::Image::ConstSharedPtr &image_msg)
  {
    if (!image_msg)
    {
      RCLCPP_WARN(get_logger(), "No image available for remote RGB recognition");
      return false;
    }

    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();

    {
      std::lock_guard<std::mutex> lock(recognized_image_sync_mutex_);
      recognized_image_list_.objects.clear();
      recognized_image_promise_ = promise;
    }

    waiting_for_recognized_image_.store(true);

    sensor_msgs::msg::Image request = *image_msg;
    request.header.stamp = this->get_clock()->now();
    pub_rgb_to_recognizer_->publish(request);

    const auto status = future.wait_for(
        std::chrono::duration<double>(rgb_recognizer_timeout_sec_));

    waiting_for_recognized_image_.store(false);

    {
      std::lock_guard<std::mutex> lock(recognized_image_sync_mutex_);
      recognized_image_promise_.reset();
    }

    if (status != std::future_status::ready)
    {
      RCLCPP_WARN(get_logger(),
                  "Timeout waiting for remote RGB recognizer response on %s",
                  rgb_recognizer_output_topic_.c_str());
      return false;
    }

    return true;
}

void
MultiModalObjectRecognitionROS::preprocessPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg)
{
  sensor_msgs::msg::PointCloud2 msg_transformed;
  msg_transformed.header.frame_id = target_frame_id_;
  if (!mpu::pointcloud::transformPointCloudMsg(
        tf_buffer_, target_frame_id_, *cloud_msg, msg_transformed)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Unable to transform pointcloud. Are you sure target_frame_id_ and "
      "pointcloud_source_frame_id are set correctly?");
    RCLCPP_ERROR(this->get_logger(),
                 "pointcloud_source_frame_id: %s, target_frame_id: %s",
                 cloud_msg->header.frame_id.c_str(),
                 target_frame_id_.c_str());
    RCLCPP_ERROR(this->get_logger(),
                 "pointcloud_source_frame_id may need to be "
                 "arm_cam3d_camera_color_frame or fixed_camera_link");
    RCLCPP_ERROR(
      this->get_logger(),
      "target_frame_id may need to be base_link or base_link_static");
    return;
  }
  std::shared_ptr<pcl::PCLPointCloud2> pc2 =
    std::make_shared<pcl::PCLPointCloud2>();
  pcl_conversions::toPCL(msg_transformed, *pc2);
  pc2->header.frame_id = msg_transformed.header.frame_id;

  cloud_ = PointCloudBSPtr(new PointCloud);
  pcl::fromPCLPointCloud2(*pc2, *cloud_);
}

void
MultiModalObjectRecognitionROS::segmentPointCloud(
  mir_interfaces::msg::ObjectList& object_list,
  std::vector<PointCloudBSPtr>& clusters,
  std::vector<mpu::object::BoundingBox>& boxes)
{
  PointCloudBSPtr cloud = PointCloudBSPtr(new PointCloud);
  cloud->header.frame_id = target_frame_id_;

  scene_segmentation_ros_->getCloudAccumulation(cloud);

  // if the cluster is centered,it looses the correct location of the object
  scene_segmentation_ros_->segmentCloud(cloud,
                                        object_list,
                                        clusters,
                                        boxes,
                                        center_cluster_ = false,
                                        pad_cluster_,
                                        padded_cluster_size_);

  // get workspace height
  std_msgs::msg::Float64 workspace_height_msg;
  workspace_height_msg.data = scene_segmentation_ros_->getWorkspaceHeight();
  pub_workspace_height_->publish(workspace_height_msg);

  if (debug_mode_) {
    PointCloudBSPtr cloud_debug(new PointCloud);
    cloud_debug = scene_segmentation_ros_->getCloudDebug();
    sensor_msgs::msg::PointCloud2 ros_pc2;
    pcl::toROSMsg(*cloud_debug, ros_pc2);
    ros_pc2.header.frame_id = target_frame_id_;
    RCLCPP_INFO_STREAM(get_logger(), "Publishing debug cloud plane");
    pub_debug_cloud_plane_->publish(ros_pc2);
  }
}

void
MultiModalObjectRecognitionROS::recognizeCloudAndImage(
  const std::string workstation)
{
  mir_interfaces::msg::ObjectList cloud_object_list;
  std::vector<PointCloudBSPtr> clusters_3d;
  std::vector<mpu::object::BoundingBox> boxes;

  // Object detection and bounding box generation

  this->segmentPointCloud(cloud_object_list, clusters_3d, boxes);

  if (debug_mode_) {
    // convert the bouinding boxes into ros message
    mir_interfaces::msg::BoundingBoxList bounding_box_list;
    bounding_box_list.bounding_boxes.resize(boxes.size());
    // loop through boxes
    for (size_t i = 0; i < boxes.size(); i++) {
      convertBoundingBox(boxes[i], bounding_box_list.bounding_boxes[i]);
    }
    bounding_box_visualizer_pc_->publish(bounding_box_list.bounding_boxes,
                                         target_frame_id_);
  }

  // publish segmented cloud object list for recognition
  if (!cloud_object_list.objects.empty() && enable_pc_recognizer_) {
    RCLCPP_INFO_STREAM(get_logger(), "Publishing pointcloud for recognition");

    pub_cloud_to_recognizer_->publish(cloud_object_list);
  }

  // RGB Object recognition

  if (recognized_image_list_.objects.empty() && enable_rgb_recognizer_) {
    RCLCPP_INFO_STREAM(get_logger(), "Performing RGB recognition");
    // convert the image_msg_ into cv::Mat
    cv_bridge::CvImagePtr image_cv_ptr;
    try {
      image_cv_ptr =
        cv_bridge::toCvCopy(image_msg_, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // pass the image to the rgb yolov inference class and get the recognized
    // objects list
    RecognizedObjectList rgb_recog_list =
      yolo_inference_->run_inference(image_cv_ptr->image);

    recognized_image_list_.objects.resize(rgb_recog_list.size());
    for (size_t i = 0; i < rgb_recog_list.size(); i++) {
      recognized_image_list_.objects[i].name = rgb_recog_list[i].class_name;
      recognized_image_list_.objects[i].probability =
        rgb_recog_list[i].confidence;
      recognized_image_list_.objects[i].roi.x_offset = rgb_recog_list[i].roi[0];
      recognized_image_list_.objects[i].roi.y_offset = rgb_recog_list[i].roi[1];
      recognized_image_list_.objects[i].roi.width = rgb_recog_list[i].roi[2];
      recognized_image_list_.objects[i].roi.height = rgb_recog_list[i].roi[3];
    }

    if (debug_mode_) {
      for (size_t i = 0; i < rgb_recog_list.size(); i++) {
        RecognizedObject obj = rgb_recog_list[i];
        int left = obj.roi[0];
        int top = obj.roi[1];
        int width = obj.roi[2];
        int height = obj.roi[3];
        string class_name = obj.class_name;
        float conf = obj.confidence;
        // Draw bounding box.
        rectangle(image_cv_ptr->image,
                  Point(left, top),
                  Point(left + width, top + height),
                  BLUE,
                  3 * THICKNESS);
        // Get the label for the class name and its confidence.
        string label = format("%.2f", conf);
        label = class_name + ":" + label;
        // Draw class labels.
        yolo_inference_->draw_label(image_cv_ptr->image, label, left, top);
      }
      vector<double> layersTimes;
      double freq = getTickFrequency() / 1000;
      double t = yolo_inference_->net.getPerfProfile(layersTimes) / freq;
      string label = format("Inference time : %.2f ms", t);
      putText(
        image_cv_ptr->image, label, Point(20, 40), FONT_FACE, FONT_SCALE, RED);

      // convert the cv::Mat into ros message and publish
      std::shared_ptr<sensor_msgs::msg::Image> image_msg =
        image_cv_ptr->toImageMsg();
      image_msg->header.frame_id = target_frame_id_;
      pub_debug_rgb_image_->publish(*image_msg);
    }
  }

  // TODO: Test PC object recognition

  mir_interfaces::msg::ObjectList combined_object_list;
  if (!recognized_cloud_list_.objects.empty()) {
    combined_object_list.objects.insert(combined_object_list.objects.end(),
                                        recognized_cloud_list_.objects.begin(),
                                        recognized_cloud_list_.objects.end());
  }

  // Reset recognition callback flags
  received_recognized_cloud_list_flag_ = false;

  mir_interfaces::msg::ObjectList rgb_object_list;
  mir_interfaces::msg::BoundingBoxList bounding_boxes;
  std::vector<PointCloudBSPtr> clusters_2d;

  cv_bridge::CvImagePtr cv_image;
  if (recognized_image_list_.objects.size() > 0) {
    try {
      cv_image =
        cv_bridge::toCvCopy(image_msg_, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    bounding_boxes.bounding_boxes.resize(recognized_image_list_.objects.size());
    rgb_object_list.objects.resize(recognized_image_list_.objects.size());

    for (size_t i = 0; i < recognized_image_list_.objects.size(); i++) {
      mir_interfaces::msg::Object object = recognized_image_list_.objects[i];
      // Check qualitative info of the object
      if (round_objects_.count(recognized_image_list_.objects[i].name)) {
        object.shape.shape = object.shape.SPHERE;
      } else {
        object.shape.shape = object.shape.OTHER;
      }
      // Get ROI
      sensor_msgs::msg::RegionOfInterest roi_2d = object.roi;
      const cv::Rect2d rect2d(
        roi_2d.x_offset, roi_2d.y_offset, roi_2d.width, roi_2d.height);

      if (debug_mode_) {
        cv::Point pt1;
        cv::Point pt2;

        pt1.x = roi_2d.x_offset;
        pt1.y = roi_2d.y_offset;
        pt2.x = roi_2d.x_offset + roi_2d.width;
        pt2.y = roi_2d.y_offset + roi_2d.height;

        // draw bbox
        cv::rectangle(
          cv_image->image, pt1, pt2, cv::Scalar(0, 255, 0), 1, 8, 0);

        // add label
        cv::putText(cv_image->image,
                    object.name,
                    cv::Point(pt1.x, pt2.y),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.3,
                    cv::Scalar(0, 255, 0),
                    1);
      }

      // Remove large 2d misdetected bbox (misdetection)
      double len_diag = sqrt(pow(roi_2d.width, 2) + pow(roi_2d.height, 2));
      if (len_diag > rgb_bbox_min_diag_ && len_diag < rgb_bbox_max_diag_) {
        PointCloudBSPtr cloud_roi(new PointCloud);
        bool getROISuccess =
          mpu::pointcloud::getPointCloudROI(roi_2d,
                                            cloud_,
                                            cloud_roi,
                                            rgb_roi_adjustment_,
                                            rgb_cluster_remove_outliers_);

        // ToDo: Filter big objects from 2d proposal, if the height is less than
        // 3 mm pcl::PointXYZRGB min_pt; pcl::PointXYZRGB max_pt;
        // pcl::getMinMax3D(*cloud_roi, min_pt, max_pt);
        // float obj_height = max_pt.z -
        // scene_segmentation_ros_->getWorkspaceHeight();

        if (getROISuccess) {
          sensor_msgs::msg::PointCloud2 ros_pc2;
          PCLPointCloud2BSPtr pc2(new pcl::PCLPointCloud2);
          pcl::toPCLPointCloud2(*cloud_roi, *pc2);
          pcl_conversions::fromPCL(*pc2, ros_pc2);
          ros_pc2.header.frame_id = target_frame_id_;
          ros_pc2.header.stamp = this->get_clock()->now();

          rgb_object_list.objects[i].views.resize(1);
          rgb_object_list.objects[i].views[0].point_cloud = ros_pc2;

          clusters_2d.push_back(cloud_roi);

          // Get pose
          geometry_msgs::msg::PoseStamped pose;
          mpu::object::estimatePose(cloud_roi,
                                    pose,
                                    object.shape.shape,
                                    rgb_cluster_filter_limit_min_,
                                    rgb_cluster_filter_limit_max_);

          // Transform pose
          std::string frame_id = cloud_->header.frame_id;
          pose.header.stamp = this->get_clock()->now();
          pose.header.frame_id = frame_id;
          if (frame_id != target_frame_id_) {
            mpu::object::transformPose(tf_buffer_,
                                       target_frame_id_,
                                       pose,
                                       rgb_object_list.objects[i].pose);
          } else {
            rgb_object_list.objects[i].pose = pose;
          }
          rgb_object_list.objects[i].probability =
            recognized_image_list_.objects[i].probability;
          rgb_object_list.objects[i].database_id = rgb_object_id_;
          rgb_object_list.objects[i].name =
            recognized_image_list_.objects[i].name;
        } else {
          RCLCPP_DEBUG(get_logger(), "[RGB] DECOY");
          rgb_object_list.objects[i].name = "DECOY";
          rgb_object_list.objects[i].database_id = rgb_object_id_;
        }
      } else {
        RCLCPP_DEBUG(get_logger(), "[RGB] DECOY");
        rgb_object_list.objects[i].name = "DECOY";
        rgb_object_list.objects[i].database_id = rgb_object_id_;
      }
      rgb_object_id_++;
    }
    combined_object_list.objects.insert(combined_object_list.objects.end(),
                                        rgb_object_list.objects.begin(),
                                        rgb_object_list.objects.end());
  }

  if (!combined_object_list.objects.empty()) {
    if (enable_roi_) {
      for (size_t i = 0; i < combined_object_list.objects.size(); i++) {
        double current_object_pose_x =
          combined_object_list.objects[i].pose.pose.position.x;
        if (current_object_pose_x < roi_base_link_to_laser_distance_ ||
            current_object_pose_x > roi_max_object_pose_x_to_base_link_)
        // combined_object_list.objects[i].pose.pose.position.z <
        // scene_segmentation_ros_
        // ->object_height_above_workspace_ - 0.05)
        {
          RCLCPP_WARN_STREAM(get_logger(),
                             "This object "
                               << combined_object_list.objects[i].name
                               << " out of RoI");
          combined_object_list.objects[i].name = "DECOY";
        }
      }
    }
    // Adjust RPY to make pose flat, adjust container pose
    // Adjust Axis and Bolt pose
    adjustObjectPose(combined_object_list);
    // Publish object to object list merger
    // add workstation info
    combined_object_list.workstation_name = workstation;
    publishObjectList(combined_object_list);
  } else {
    RCLCPP_WARN(get_logger(), "No object detected to publish");
    return;
  }

  if (debug_mode_) {
    RCLCPP_WARN_STREAM(get_logger(),
                       "Debug mode: publishing object information");
    publishDebug(combined_object_list, clusters_3d, clusters_2d);
  }
}

void
MultiModalObjectRecognitionROS::adjustObjectPose(
  mir_interfaces::msg::ObjectList& object_list)
{
  for (size_t i = 0; i < object_list.objects.size(); i++) {
    tf2::Quaternion q(object_list.objects[i].pose.pose.orientation.x,
                      object_list.objects[i].pose.pose.orientation.y,
                      object_list.objects[i].pose.pose.orientation.z,
                      object_list.objects[i].pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double change_in_pitch = 0.0;
    if (round_objects_.count(object_list.objects[i].name)) {
      yaw = 0.0;
    }
    // Update container pose
    if (object_list.objects[i].name == "CONTAINER_BOX_RED" ||
        object_list.objects[i].name == "CONTAINER_BOX_BLUE") {
      if (object_list.objects[i].database_id > 100) {
        RCLCPP_DEBUG_STREAM(get_logger(), "Updating RGB container pose");
        mm_object_recognition_utils_->adjustContainerPose(
          object_list.objects[i], container_height_);
      }
    }
    // Make pose flat
    tf2::Quaternion q2;
    q2.setRPY(0.0, change_in_pitch, yaw);

    object_list.objects[i].pose.pose.orientation.x = q2.x();
    object_list.objects[i].pose.pose.orientation.y = q2.y();
    object_list.objects[i].pose.pose.orientation.z = q2.z();
    object_list.objects[i].pose.pose.orientation.w = q2.w();

    // Update workspace height
    if (scene_segmentation_ros_->getWorkspaceHeight() != -1000.0) {
      object_list.objects[i].pose.pose.position.z =
        scene_segmentation_ros_->getWorkspaceHeight() +
        object_height_above_workspace_;

      if (object_list.objects[i].name == "CONTAINER_BOX_RED" ||
          object_list.objects[i].name == "CONTAINER_BOX_BLUE") {
        object_list.objects[i].pose.pose.position.z =
          scene_segmentation_ros_->getWorkspaceHeight() + container_height_;
        RCLCPP_WARN_STREAM(get_logger(),
                           "Updated container height: "
                             << object_list.objects[i].pose.pose.position.z);
      }
    }
    // Update axis or bolt pose
    if (object_list.objects[i].name == "M20_100" ||
        object_list.objects[i].name == "AXIS") {
      mm_object_recognition_utils_->adjustAxisBoltPose(object_list.objects[i]);
    }
  }
}

void
MultiModalObjectRecognitionROS::publishObjectList(
  mir_interfaces::msg::ObjectList& object_list)
{
  for (size_t i = 0; i < object_list.objects.size(); i++) {
    // Empty cloud
    sensor_msgs::msg::PointCloud2 empty_ros_cloud;
    object_list.objects[i].views.resize(1);
    object_list.objects[i].views[0].point_cloud = empty_ros_cloud;
    // Rename container to match refbox naming
    if (object_list.objects[i].name == "BLUE_CONTAINER") {
      object_list.objects[i].name = "CONTAINER_BOX_BLUE";
    } else if (object_list.objects[i].name == "RED_CONTAINER") {
      object_list.objects[i].name = "CONTAINER_BOX_RED";
    }
  }
  // Publish object list to object list merger
  pub_object_list_->publish(object_list);
}

void
MultiModalObjectRecognitionROS::publishDebug(
  mir_interfaces::msg::ObjectList& combined_object_list,
  std::vector<PointCloudBSPtr>& clusters_3d,
  std::vector<PointCloudBSPtr>& clusters_2d)
{
  RCLCPP_INFO_STREAM(get_logger(),
                     "Cloud list: " << recognized_cloud_list_.objects.size());
  RCLCPP_INFO_STREAM(get_logger(),
                     "RGB list: " << recognized_image_list_.objects.size());
  RCLCPP_INFO_STREAM(
    get_logger(),
    "Combined object list: " << combined_object_list.objects.size());

  // Compute normal to generate parallel BBOX to the plane
  const Eigen::Vector3f normal = scene_segmentation_ros_->getPlaneNormal();

  std::string names = "";
  if (recognized_cloud_list_.objects.size() > 0) {
    // Bounding boxes
    if (clusters_3d.size() > 0) {
      mir_interfaces::msg::BoundingBoxList bounding_boxes;
      cluster_visualizer_pc_->publish(clusters_3d, target_frame_id_);
      bounding_boxes.bounding_boxes.resize(clusters_3d.size());
      for (size_t i = 0; i < clusters_3d.size(); i++) {
        mpu::object::BoundingBox bbox;
        mpu::object::get3DBoundingBox(
          clusters_3d[i], normal, bbox, bounding_boxes.bounding_boxes[i]);
      }
      if (bounding_boxes.bounding_boxes.size() > 0) {
        bounding_box_visualizer_pc_->publish(bounding_boxes.bounding_boxes,
                                             target_frame_id_);
      }
    }
    // PCL Pose array for debug mode only
    geometry_msgs::msg::PoseArray pcl_object_pose_array;
    pcl_object_pose_array.header.frame_id = target_frame_id_;
    pcl_object_pose_array.header.stamp = this->get_clock()->now();
    pcl_object_pose_array.poses.resize(recognized_cloud_list_.objects.size());
    std::vector<std::string> pcl_labels;
    int pcl_count = 0;
    for (size_t i = 0; i < combined_object_list.objects.size(); i++) {
      if (combined_object_list.objects[i].database_id < 99) {
        names += combined_object_list.objects[i].name + ", ";
        pcl_object_pose_array.poses[pcl_count] =
          combined_object_list.objects[i].pose.pose;
        pcl_labels.push_back(combined_object_list.objects[i].name);
        pcl_count++;
      }
    }
    RCLCPP_INFO_STREAM(get_logger(), "[Cloud] Objects: " << names);
    // Publish pose array
    if (pcl_object_pose_array.poses.size() > 0) {
      pub_pc_object_pose_array_->publish(pcl_object_pose_array);
    }
    // Publish label visualizer
    if ((pcl_labels.size() == pcl_object_pose_array.poses.size()) &&
        (pcl_labels.size() > 0) && (pcl_object_pose_array.poses.size() > 0)) {
      label_visualizer_pc_->publish(pcl_labels, pcl_object_pose_array);
    }
  }
  if (clusters_2d.size() > 0) {
    cluster_visualizer_rgb_->publish(clusters_2d, target_frame_id_);
    // RGB Pose array for debug mode only
    geometry_msgs::msg::PoseArray rgb_object_pose_array;
    rgb_object_pose_array.header.frame_id = target_frame_id_;
    rgb_object_pose_array.header.stamp = this->get_clock()->now();
    rgb_object_pose_array.poses.resize(recognized_image_list_.objects.size());
    std::vector<std::string> rgb_labels;
    int rgb_count = 0;
    names = "";
    for (size_t i = 0; i < combined_object_list.objects.size(); i++) {
      if (combined_object_list.objects[i].database_id > 99) {
        names += combined_object_list.objects[i].name + ", ";
        rgb_object_pose_array.poses[rgb_count] =
          combined_object_list.objects[i].pose.pose;
        rgb_labels.push_back(combined_object_list.objects[i].name);
        rgb_count++;
      }
    }
    RCLCPP_INFO_STREAM(get_logger(), "[RGB] Objects: " << names);
    // Publish pose array
    if (rgb_object_pose_array.poses.size() > 0) {
      RCLCPP_INFO_STREAM(get_logger(), "[RGB] Publishing pose array");
      pub_rgb_object_pose_array_->publish(rgb_object_pose_array);
    }
    // Publish label visualizer
    if ((rgb_labels.size() == rgb_object_pose_array.poses.size()) &&
        (rgb_labels.size() > 0) && (rgb_object_pose_array.poses.size() > 0)) {
      label_visualizer_rgb_->publish(rgb_labels, rgb_object_pose_array);
    }
  }
}

void
MultiModalObjectRecognitionROS::loadObjectInfo(const std::string& filename)
{
  YAML::Node config = YAML::LoadFile(filename);
  mir_interfaces::msg::Object object1;
  if (config["object_info"]) {
    for (unsigned j = 0; j < config["object_info"]["object"].size(); ++j) {
      Object f;
      f.name = config["object_info"]["object"][j]["name"].as<std::string>();
      f.shape = config["object_info"]["object"][j]["shape"].as<std::string>();
      f.color = config["object_info"]["object"][j]["color"].as<std::string>();
      // RCLCPP_INFO(get_logger(), "%s || %s || %s", f.name.c_str(),
      // f.shape.c_str(), f.color.c_str());
      if (f.shape == object1.shape.SPHERE) {
        round_objects_.insert(f.name);
        // RCLCPP_INFO(get_logger(), "Round object detected !");
      }
      object_info_.push_back(f);
    }
    RCLCPP_INFO(get_logger(), "Object info is loaded!");
  } else {
    RCLCPP_WARN(get_logger(), "No object info is provided!");
  }
}

rclcpp_action::GoalResponse
MultiModalObjectRecognitionROS::handle_goal(
  const rclcpp_action::GoalUUID& uuid,
  std::shared_ptr<const mir_interfaces::action::ObjectDetection::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(),
              "Received goal request with object name: %s",
              goal->obj_name.c_str());
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
MultiModalObjectRecognitionROS::handle_cancel(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<mir_interfaces::action::ObjectDetection>>
    goal_handle)
{
  // TODO: Handle cancel request
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
MultiModalObjectRecognitionROS::handle_accepted(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<mir_interfaces::action::ObjectDetection>>
    goal_handle)
{
  current_goal_handle_ = goal_handle;

  // execute goal
  std::thread{ std::bind(
                 &MultiModalObjectRecognitionROS::execute_goal, this, _1),
               goal_handle }
    .detach();
}

void
MultiModalObjectRecognitionROS::execute_goal(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<mir_interfaces::action::ObjectDetection>>
    goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  auto result =
    std::make_shared<mir_interfaces::action::ObjectDetection::Result>();

  auto workstation = goal_handle->get_goal()->workstation;

  // check if data is already locked
  if (data_lock_.owns_lock()) {
    RCLCPP_WARN(get_logger(),
                "Skipping goal because a previous goal is still processing "
                "data");
    result->result = false;
    goal_handle->abort(result);
    current_goal_handle_.reset();
    return;
  }

  if (pointcloud_msg_ == nullptr || image_msg_ == nullptr) {
    RCLCPP_WARN(get_logger(),
                "Skipping goal because no data has been received yet");
    result->result = false;
    goal_handle->abort(result);
    current_goal_handle_.reset();
    return;
  }

  // lock data if not locked
  if(data_lock_.try_lock()) {
    RCLCPP_INFO(get_logger(), "Locking data");
  } else {
    RCLCPP_WARN(get_logger(),
                "Skipping goal because a previous goal is still processing "
                "data");
    result->result = false;
    goal_handle->abort(result);
    current_goal_handle_.reset();
    return;
  }

    if (enable_rgb_recognizer_ && !requestRemoteRgbRecognition(image_msg_))
    {
      result->result = false;
      goal_handle->abort(result);
      current_goal_handle_.reset();
      data_lock_.unlock();
    return;
  }

  // pre-process the pointcloud
  this->preprocessPointCloud(pointcloud_msg_);
  scene_segmentation_ros_->addCloudAccumulation(cloud_);
  this->recognizeCloudAndImage(workstation);

  // Reset received recognized cloud and image
  received_recognized_cloud_list_flag_ = false;

  // reset object id
  scene_segmentation_ros_->resetPclObjectId();

  // clear recognized image and cloud list
  recognized_image_list_.objects.clear();
  recognized_cloud_list_.objects.clear();

  scene_segmentation_ros_->resetCloudAccumulation();

  // reset cloud and image
  pointcloud_msg_.reset();
  image_msg_.reset();

  // unlock data so that other goals can use it
  data_lock_.unlock();

  // TODO: check if there are any objects in the object list
  // if not, return false else return true

  if (rclcpp::ok()) {
    result->result = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    current_goal_handle_.reset();
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MultiModalObjectRecognitionROS::on_configure(const rclcpp_lifecycle::State&)
{
  // This callback is supposed to be used for initialization and
  // configuring purposes.

  RCLCPP_INFO(get_logger(), "on_configure() is called.");

  callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&MultiModalObjectRecognitionROS::parametersCallback,
              this,
              std::placeholders::_1));

  MultiModalObjectRecognitionROS::get_all_parameters();
  MultiModalObjectRecognitionROS::loadObjectInfo(objects_info_path_);

  // initialize action server
  action_server_ =
    rclcpp_action::create_server<mir_interfaces::action::ObjectDetection>(
      shared_from_this(),
      "object_detection",
      std::bind(&MultiModalObjectRecognitionROS::handle_goal, this, _1, _2),
      std::bind(&MultiModalObjectRecognitionROS::handle_cancel, this, _1),
      std::bind(&MultiModalObjectRecognitionROS::handle_accepted, this, _1));

  // // initialize yolo_inference
  // yolo_inference_ = std::make_shared<YoloInference>(yolo_weights_path_,
  //                                                   yolo_classes_info_path_);

  // declare qos profile
  qos_sensor = rclcpp::SensorDataQoS(rclcpp::KeepLast(10));
  qos_parameters = rclcpp::ParametersQoS(rclcpp::KeepLast(1));
  qos_default = rclcpp::SystemDefaultsQoS(rclcpp::KeepLast(10));

  // initializing variables
  rgb_object_id_ = 100;
  container_height_ = 0.05;
  received_recognized_cloud_list_flag_ = false;
  enable_rgb_recognizer_ = true;
  enable_pc_recognizer_ = false;
  rgb_roi_adjustment_ = 2;
  rgb_cluster_remove_outliers_ = true;

  msg_sync_ = std::make_shared<Sync>(msgSyncPolicy(10), image_sub_, cloud_sub_);

  // create data lock
  data_lock_ = std::unique_lock<std::mutex>(data_mutex_, std::defer_lock);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // publish workspace height
  pub_workspace_height_ = this->create_publisher<std_msgs::msg::Float64>(
    "~/workspace_height", qos_parameters);

  // publish debug
  pub_debug_cloud_plane_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/output/debug_cloud_plane", qos_sensor);

  // Publish cloud and images to cloud and rgb recognition topics
  pub_cloud_to_recognizer_ =
    this->create_publisher<mir_interfaces::msg::ObjectList>(
      "recognizer/pc/input/object_list", qos_parameters);

  // rgb image debug publisher
  pub_debug_rgb_image_ = this->create_publisher<sensor_msgs::msg::Image>(
    "~/recognizer/rgb/debug_image", qos_default);

  // Remote RGB recognizer request publisher
  pub_rgb_to_recognizer_ = this->create_publisher<sensor_msgs::msg::Image>(
      rgb_recognizer_input_topic_, qos_sensor);

  // Subscribe to cloud and rgb recognition topics
  recognized_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  recognized_sub_options = rclcpp::SubscriptionOptions();
  recognized_sub_options.callback_group = recognized_callback_group_;

  sub_recognized_cloud_list_ =
    this->create_subscription<mir_interfaces::msg::ObjectList>(
      "recognizer/pc/output/object_list",
      qos_parameters,
      std::bind(&MultiModalObjectRecognitionROS::recognizedCloudCallback,
                this,
                std::placeholders::_1),
      recognized_sub_options);

    sub_recognized_image_list_ =
        this->create_subscription<mir_interfaces::msg::ObjectList>(
            rgb_recognizer_output_topic_,
            qos_parameters,
            std::bind(&MultiModalObjectRecognitionROS::recognizedImageCallback,
                this,
                std::placeholders::_1),
      recognized_sub_options);

  // publish pose arrays
  pub_pc_object_pose_array_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>(
      "~/output/pc_object_pose_array", qos_default);
  pub_rgb_object_pose_array_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>(
      "~/output/rgb_object_pose_array", qos_default);

  // Pub combined object_list to object_list merger
  pub_object_list_ = this->create_publisher<mir_interfaces::msg::ObjectList>(
    "/world_model_node/object_list", qos_parameters);

  // initialize bounding box visualizer
  bounding_box_visualizer_pc_ = std::make_shared<BoundingBoxVisualizer>(
    shared_from_this(), "~/output/bounding_boxes", Color(Color::IVORY));

  // initialize cluster visualizer for rgb and pc
  cluster_visualizer_rgb_ = std::make_shared<ClusteredPointCloudVisualizer>(
    shared_from_this(), "~/output/tabletop_cluster_rgb", true);
  cluster_visualizer_pc_ = std::make_shared<ClusteredPointCloudVisualizer>(
    shared_from_this(), "~/output/tabletop_cluster_pc");

  // initialize label visualizer for rgb and pc
  label_visualizer_rgb_ = std::make_shared<LabelVisualizer>(
    shared_from_this(), "~/output/rgb_labels", Color(Color::SEA_GREEN));
  label_visualizer_pc_ = std::make_shared<LabelVisualizer>(
    shared_from_this(), "~/output/pc_labels", Color(Color::IVORY));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MultiModalObjectRecognitionROS::on_activate(const rclcpp_lifecycle::State&)
{
  // Activate publishers and other services here

  RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

  image_sub_.subscribe(this, "input_image_topic");
  cloud_sub_.subscribe(this, "input_cloud_topic");

  // register msg sync callback
  msg_sync_->registerCallback(
    &MultiModalObjectRecognitionROS::synchronizeCallback, this);

  pub_workspace_height_->on_activate();
  pub_debug_cloud_plane_->on_activate();
  pub_debug_rgb_image_->on_activate();
  pub_rgb_to_recognizer_->on_activate();
  pub_cloud_to_recognizer_->on_activate();
  pub_object_list_->on_activate();
  pub_pc_object_pose_array_->on_activate();
  pub_rgb_object_pose_array_->on_activate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MultiModalObjectRecognitionROS::on_deactivate(const rclcpp_lifecycle::State&)
{
  // Deactivate publishers and subscribers

  RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

  pub_workspace_height_->on_deactivate();
  pub_debug_cloud_plane_->on_deactivate();
  pub_debug_rgb_image_->on_deactivate();
  pub_rgb_to_recognizer_->on_deactivate();
  pub_cloud_to_recognizer_->on_deactivate();
  pub_object_list_->on_deactivate();
  pub_pc_object_pose_array_->on_deactivate();
  pub_rgb_object_pose_array_->on_deactivate();

  msg_sync_.reset(new Sync(msgSyncPolicy(10), image_sub_, cloud_sub_));

  image_sub_.unsubscribe();
  cloud_sub_.unsubscribe();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MultiModalObjectRecognitionROS::on_cleanup(const rclcpp_lifecycle::State&)
{
  // Release all pointers and reset variables

  RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

  image_sub_.unsubscribe();
  cloud_sub_.unsubscribe();
  msg_sync_.reset();
  tf_buffer_.reset();
  tf_listener_.reset();

  pub_workspace_height_.reset();
  pub_debug_cloud_plane_.reset();
  pub_debug_rgb_image_.reset();
  pub_rgb_to_recognizer_.reset();
  pub_cloud_to_recognizer_.reset();
  pub_object_list_.reset();
  pub_pc_object_pose_array_.reset();
  pub_rgb_object_pose_array_.reset();

  sub_recognized_cloud_list_.reset();
  sub_recognized_image_list_.reset();

  bounding_box_visualizer_pc_.reset();
  cluster_visualizer_rgb_.reset();
  cluster_visualizer_pc_.reset();
  label_visualizer_rgb_.reset();
  label_visualizer_pc_.reset();

  recognized_callback_group_.reset();

  this->remove_on_set_parameters_callback(callback_handle_.get());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MultiModalObjectRecognitionROS::on_shutdown(
  const rclcpp_lifecycle::State& state)
{
  // Release all pointers and reset variables

  RCUTILS_LOG_INFO_NAMED(
    get_name(), "on shutdown is called from state %s.", state.label().c_str());

  image_sub_.unsubscribe();
  cloud_sub_.unsubscribe();
  msg_sync_.reset();
  tf_buffer_.reset();
  tf_listener_.reset();

  pub_workspace_height_.reset();
  pub_debug_cloud_plane_.reset();
  pub_debug_rgb_image_.reset();
  pub_rgb_to_recognizer_.reset();
  pub_cloud_to_recognizer_.reset();
  pub_object_list_.reset();
  pub_pc_object_pose_array_.reset();
  pub_rgb_object_pose_array_.reset();

  sub_recognized_cloud_list_.reset();
  sub_recognized_image_list_.reset();

  bounding_box_visualizer_pc_.reset();
  cluster_visualizer_rgb_.reset();
  cluster_visualizer_pc_.reset();
  label_visualizer_rgb_.reset();
  label_visualizer_pc_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

} // end of namespace

// Registering the node as a component
RCLCPP_COMPONENTS_REGISTER_NODE(
  perception_namespace::MultiModalObjectRecognitionROS)