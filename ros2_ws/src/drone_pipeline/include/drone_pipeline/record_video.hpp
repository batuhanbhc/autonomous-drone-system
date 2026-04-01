// record_video.hpp 
#ifndef DRONE_PIPELINE__RECORD_VIDEO_HPP_
#define DRONE_PIPELINE__RECORD_VIDEO_HPP_

#include <atomic>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "drone_msgs/msg/frame_data.hpp"
#include "drone_msgs/msg/toggle.hpp"
#include "drone_pipeline/mjpeg_writer.hpp"

namespace drone_pipeline
{

struct VideoConfig
{
  uint8_t     drone_id{};
  std::string frames_topic;
  std::string logs_path;
  int         width{};
  int         height{};
  int         fps{};
};

class RecordVideo : public rclcpp::Node
{
public:
  explicit RecordVideo(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RecordVideo();

private:
  VideoConfig loadConfig();
  std::string resolveSessionDir(const std::string & logs_path);

  void openClip();
  void closeClip();
  void startRecordThread();
  void stopRecordThread();
  void recordLoop();

  void publishRecordState();
  void recordCmdCallback(const drone_msgs::msg::Toggle::SharedPtr msg);
  void frameCallback(drone_msgs::msg::FrameData::ConstSharedPtr msg);

  // ── Config ────────────────────────────────────────────────
  VideoConfig config_;
  std::string session_dir_;
  std::string videos_dir_;
  std::string data_dir_;

  // ── ROS interfaces ────────────────────────────────────────
  rclcpp::Subscription<drone_msgs::msg::FrameData>::SharedPtr frame_sub_;
  rclcpp::Subscription<drone_msgs::msg::Toggle>::SharedPtr    record_cmd_sub_;
  rclcpp::Publisher<drone_msgs::msg::Toggle>::SharedPtr       record_state_pub_;

  // ── Recording state ───────────────────────────────────────
  struct RecordTask
  {
    drone_msgs::msg::FrameData::ConstSharedPtr msg;
  };


  static constexpr std::size_t kMaxQueueDepth = 60;

  std::atomic<bool>       recording_{false};
  int                     clip_index_{0};

  MjpegWriter             mjpeg_writer_;
  std::ofstream           csv_file_;
  std::mutex              clip_mtx_;

  std::thread             record_thread_;
  std::atomic<bool>       record_thread_running_{false};
  std::queue<RecordTask>  record_queue_;
  std::mutex              record_queue_mtx_;
  std::condition_variable record_queue_cv_;
};

}  // namespace drone_pipeline
#endif  // DRONE_PIPELINE__RECORD_VIDEO_HPP_