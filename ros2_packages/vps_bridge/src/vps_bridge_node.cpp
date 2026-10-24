#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/header.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

// Bridges whichever VIO/VSLAM estimator is running (cuVSLAM or OpenVINS) to
// MAVROS's vision_pose input. Only one estimator is expected to be publishing
// at a time; both subscriptions feed the same downstream publisher.
//
// Before forwarding, each pose is sanity-checked against the previous
// accepted pose from the same source: an implausible instantaneous velocity
// or a covariance spike gets rejected instead of reaching the flight
// controller. This exists because cuVSLAM has a known failure mode
// (single-frame catastrophic pose jumps, seen up to ~90 m/s, that its own
// `vo_state` flag does not reliably detect) — see docs/KNOWN_ISSUES.md.
// Rejected updates are simply dropped (last good pose stays in effect
// downstream) rather than forwarded.
class VpsBridgeNode : public rclcpp::Node {
public:
    VpsBridgeNode() : Node("vps_bridge_node") {
        max_velocity_mps_ = this->declare_parameter<double>("max_velocity_mps", 15.0);
        max_position_variance_ = this->declare_parameter<double>("max_position_variance", 0.5);
        consecutive_reject_warn_threshold_ = this->declare_parameter<int>("consecutive_reject_warn_threshold", 10);

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);

        cuvslam_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/visual_slam/tracking/odometry", 10,
            std::bind(&VpsBridgeNode::odom_callback, this, std::placeholders::_1));

        openvins_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/ov_msckf/poseimu", 10,
            std::bind(&VpsBridgeNode::openvins_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "VPS Bridge initialized. Translating cuVSLAM/OpenVINS to MAVROS. "
            "Sanity filter: max_velocity_mps=%.1f, max_position_variance=%.3f",
            max_velocity_mps_, max_position_variance_);
    }

private:
    // Tracks the last accepted pose for one estimator source, so consecutive
    // updates are checked against that source's own history (cuVSLAM and
    // OpenVINS are never mixed together in one check).
    struct SourceState {
        bool has_last = false;
        geometry_msgs::msg::Point last_position;
        rclcpp::Time last_stamp;
        int consecutive_rejects = 0;
    };

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        process_pose(msg->header, msg->pose.pose, msg->pose.covariance, cuvslam_state_, "cuVSLAM");
    }

    void openvins_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        process_pose(msg->header, msg->pose.pose, msg->pose.covariance, openvins_state_, "OpenVINS");
    }

    void process_pose(
        const std_msgs::msg::Header & header,
        const geometry_msgs::msg::Pose & pose,
        const std::array<double, 36> & covariance,
        SourceState & state,
        const char * source_name)
    {
        // Diagonal covariance layout is [x, y, z, rot_x, rot_y, rot_z] on a
        // 6x6 row-major matrix, so x/y/z variances sit at indices 0, 7, 14.
        const double position_variance = std::max({covariance[0], covariance[7], covariance[14]});
        if (position_variance > max_position_variance_) {
            std::ostringstream reason;
            reason << "position covariance " << position_variance
                   << " exceeds max " << max_position_variance_;
            reject(state, source_name, reason.str());
            return;
        }

        const rclcpp::Time stamp(header.stamp);
        if (state.has_last) {
            const double dt = (stamp - state.last_stamp).seconds();
            if (dt > 1e-4) {  // ignore duplicate/out-of-order timestamps
                const double dx = pose.position.x - state.last_position.x;
                const double dy = pose.position.y - state.last_position.y;
                const double dz = pose.position.z - state.last_position.z;
                const double velocity = std::sqrt(dx * dx + dy * dy + dz * dz) / dt;

                if (velocity > max_velocity_mps_) {
                    std::ostringstream reason;
                    reason << "implied velocity " << velocity << " m/s exceeds max "
                           << max_velocity_mps_ << " m/s (dt=" << dt << "s)";
                    reject(state, source_name, reason.str());
                    return;
                }
            }
        }

        // Accepted: forward to MAVROS and update this source's history.
        state.has_last = true;
        state.last_position = pose.position;
        state.last_stamp = stamp;
        state.consecutive_rejects = 0;

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = header;
        pose_msg.header.frame_id = "odom";
        pose_msg.pose = pose;
        publisher_->publish(pose_msg);
    }

    void reject(SourceState & state, const char * source_name, const std::string & reason) {
        state.consecutive_rejects++;
        RCLCPP_WARN(this->get_logger(), "[%s] Rejected pose update: %s", source_name, reason.c_str());
        if (state.consecutive_rejects == consecutive_reject_warn_threshold_) {
            RCLCPP_ERROR(this->get_logger(),
                "[%s] %d consecutive pose updates rejected - possible tracking failure.",
                source_name, state.consecutive_rejects);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cuvslam_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr openvins_subscription_;

    double max_velocity_mps_;
    double max_position_variance_;
    int consecutive_reject_warn_threshold_;

    SourceState cuvslam_state_;
    SourceState openvins_state_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VpsBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
