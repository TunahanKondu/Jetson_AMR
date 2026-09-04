#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include <optional>

using std::placeholders::_1;

class TicksToOdomNode : public rclcpp::Node
{
public:
  TicksToOdomNode() : Node("ticks_to_odom_node")
  {
    // --- Parametreler ---
    wheel_radius_      = this->declare_parameter("wheel_radius",       0.10);   // [m]
    wheel_separation_  = this->declare_parameter("wheel_separation",   0.63);   // [m]
    ticks_per_rev_     = this->declare_parameter("ticks_per_rev",      5000.0); // [ticks/rev]
    left_sign_         = this->declare_parameter("left_sign",          1.0);    // sol teker yön düzeltmesi (±1)
    right_sign_        = this->declare_parameter("right_sign",         1.0);    // sağ teker yön düzeltmesi (±1)
    odom_frame_        = this->declare_parameter("odom_frame",         std::string("odom"));
    base_frame_        = this->declare_parameter("base_frame",         std::string("base_footprint"));
    odom_topic_        = this->declare_parameter("odom_topic",         std::string("odom"));
    ticks_topic_       = this->declare_parameter("ticks_topic",        std::string("wheel_ticks"));
    publish_tf_        = this->declare_parameter("publish_tf",         true);
    deadband_lin_      = this->declare_parameter("deadband_lin",       0.001);  // [m/s]
    deadband_ang_      = this->declare_parameter("deadband_ang",       0.001);  // [rad/s]

    // Yayıncılar
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Abonelik
    ticks_sub_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
      ticks_topic_, 50, std::bind(&TicksToOdomNode::onTicks, this, _1));

    RCLCPP_INFO(get_logger(),
      "ticks_to_odom_node started. R=%.3f m, B=%.3f m, ticks/rev=%.1f, topic='%s' -> odom='%s', frames: %s->%s",
      wheel_radius_, wheel_separation_, ticks_per_rev_, ticks_topic_.c_str(),
      odom_topic_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  // Parametreler
  double wheel_radius_, wheel_separation_, ticks_per_rev_;
  double left_sign_, right_sign_;
  std::string odom_frame_, base_frame_, odom_topic_, ticks_topic_;
  bool publish_tf_;
  double deadband_lin_, deadband_ang_;

  // Durum
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr ticks_sub_;

  std::optional<int64_t> last_ticks_l_, last_ticks_r_;
  rclcpp::Time last_time_;
  double x_{0.0}, y_{0.0}, yaw_{0.0};

  static double normalizeAngle(double a) {
    while (a >  M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
  }

  void onTicks(const std_msgs::msg::Int64MultiArray &msg)
  {
    // Beklenen: data.size() >= 2, [left, right]
    if (msg.data.size() < 2) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "wheel_ticks must have at least 2 elements: [left, right]");
      return;
    }

    const int64_t ticks_l = msg.data[0];
    const int64_t ticks_r = msg.data[1];

    const rclcpp::Time stamp = this->now(); // Int64MultiArray header yok, node zamanı kullan

    // İlk paket: yalnızca referans al
    if (!last_ticks_l_.has_value() || !last_ticks_r_.has_value()) {
      last_ticks_l_ = ticks_l;
      last_ticks_r_ = ticks_r;
      last_time_    = stamp;
      return;
    }

    // Delta hesapla
    const int64_t dL_ticks = ticks_l - *last_ticks_l_;
    const int64_t dR_ticks = ticks_r - *last_ticks_r_;
    const double dt = (stamp - last_time_).seconds();

    // Zaman/doğruluk kontrolleri
    if (dt <= 0.0 || dt > 1.0) {  // 1 s'den büyük sıçramaları atla
      last_ticks_l_ = ticks_l;
      last_ticks_r_ = ticks_r;
      last_time_    = stamp;
      return;
    }

    last_ticks_l_ = ticks_l;
    last_ticks_r_ = ticks_r;
    last_time_    = stamp;

    // Ticks -> rad/s
    // d(rev) = d(ticks) / ticks_per_rev
    // d(rad) = d(rev) * 2π
    const double dL_rad = (static_cast<double>(dL_ticks) * 2.0 * M_PI) / ticks_per_rev_;
    const double dR_rad = (static_cast<double>(dR_ticks) * 2.0 * M_PI) / ticks_per_rev_;
    const double wl = left_sign_  * (dL_rad / dt);   // rad/s
    const double wr = right_sign_ * (dR_rad / dt);   // rad/s

    // Teker doğrusal hızları
    const double v_l = wl * wheel_radius_;
    const double v_r = wr * wheel_radius_;

    // Gövde doğrusal ve açısal hız
    double v = 0.5 * (v_r + v_l);
    double w = (v_r - v_l) / wheel_separation_;
    //double w = (v_l - v_r) / wheel_separation_;

    // Deadband
    if (std::fabs(v) < deadband_lin_) v = 0.0;
    if (std::fabs(w) < deadband_ang_) w = 0.0;

    // Unicycle modeli ile integrasyon
    const double dx = (std::fabs(w) < 1e-6)
      ? v * dt * std::cos(yaw_)
      : (v / w) * (std::sin(yaw_ + w*dt) - std::sin(yaw_));
    const double dy = (std::fabs(w) < 1e-6)
      ? v * dt * std::sin(yaw_)
      : (v / w) * (-std::cos(yaw_ + w*dt) + std::cos(yaw_));
    const double dyaw = w * dt;

    x_   += dx;
    y_   += dy;
    yaw_  = normalizeAngle(yaw_ + dyaw);

    publishOdomAndTF(stamp, v, w);
  }

  void publishOdomAndTF(const rclcpp::Time& stamp, double v, double w)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id  = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q; q.setRPY(0, 0, yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // Basit kovaryanslar (ihtiyaca göre ayarla)
    for (double &c : odom.pose.covariance) c = 0.0;
    odom.pose.covariance[0]  = 0.02; // x
    odom.pose.covariance[7]  = 0.02; // y
    odom.pose.covariance[35] = 0.04; // yaw

    odom.twist.twist.linear.x  = v;
    odom.twist.twist.linear.y  = 0.0;
    odom.twist.twist.angular.z = w;

    odom_pub_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id  = base_frame_;
      tf.transform.translation.x = x_;
      tf.transform.translation.y = y_;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(tf);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TicksToOdomNode>());
  rclcpp::shutdown();
  return 0;
}
