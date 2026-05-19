// Copyright 2023 CMP Engineers Pty Ltd
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "ublox_nav_sat_fix_hp_node/visibility_control.h"
#include "ublox_ubx_msgs/msg/gps_fix.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_cov.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_hp_pos_llh.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_status.hpp"

using std::placeholders::_1;

// size of position covariance array
static const size_t POS_COV_ARR_SIZE = 9;

namespace ublox_nav_sat_fix_hp
{

class UbloxNavSatHpFixNode : public rclcpp::Node
{
public:
  UBLOX_NAV_SAT_FIX_HP_NODE_PUBLIC
  explicit UbloxNavSatHpFixNode(const rclcpp::NodeOptions & options)
  : Node("ublox_nav_sat_fix_hp",
      rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
  {
    RCLCPP_INFO(this->get_logger(), "starting %s", get_name());

    cached_enu_pos_cov_.fill(0.0);

    auto qos = rclcpp::SensorDataQoS();
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

    // Create publishers
    nav_sat_fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix", qos, pub_options);

    // Create subscribers
    ubx_nav_hp_pos_llh_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavHPPosLLH>(
      "ubx_nav_hp_pos_llh", qos,
      std::bind(&UbloxNavSatHpFixNode::nav_hp_pos_llh_callback, this, std::placeholders::_1));
    ubx_nav_cov_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavCov>(
      "ubx_nav_cov", qos,
      std::bind(&UbloxNavSatHpFixNode::nav_cov_callback, this, std::placeholders::_1));
    ubx_nav_status_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavStatus>(
      "ubx_nav_status", qos,
      std::bind(&UbloxNavSatHpFixNode::nav_sta_callback, this, std::placeholders::_1));
  }

  UBLOX_NAV_SAT_FIX_HP_NODE_LOCAL
  ~UbloxNavSatHpFixNode() {RCLCPP_INFO(this->get_logger(), "finished");}

private:
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_pub_;

  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavHPPosLLH>::SharedPtr ubx_nav_hp_pos_llh_sub_;
  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavCov>::SharedPtr ubx_nav_cov_sub_;
  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavStatus>::SharedPtr ubx_nav_status_sub_;

  // Cached values from the NAV-COV and NAV-STATUS streams. These two messages
  // are emitted at the end of the F9P's per-epoch UBX output sequence and are
  // routinely dropped by the receiver's internal USB tx buffer when bigger
  // messages (NAV-SAT, NAV-SIG, NMEA-GGA) co-occur — in practice we observed
  // NAV-STATUS arriving at ~3 Hz and NAV-COV at ~1 Hz when configured at 7 Hz.
  // Each cached value is tagged with the iTOW of the epoch it came from so we
  // only apply it to /fix when it matches the current HPPOSLLH epoch; older
  // values are not republished blindly (which silently labelled Float epochs
  // as Fixed and vice-versa).
  std::array<double, POS_COV_ARR_SIZE> cached_enu_pos_cov_;
  uint32_t cached_cov_itow_ = 0;
  bool have_cached_cov_ = false;

  sensor_msgs::msg::NavSatStatus cached_nav_sat_stat_;
  uint32_t cached_status_itow_ = 0;
  bool have_cached_status_ = false;

  UBLOX_NAV_SAT_FIX_HP_NODE_LOCAL
  void nav_hp_pos_llh_callback(
    const ublox_ubx_msgs::msg::UBXNavHPPosLLH::SharedPtr ubx_hppos_llh_msg)
  {
    // Drop epochs the receiver flagged as invalid. Without this gate the HP
    // node would still publish a (likely bogus) lat/lon, attached to whatever
    // status was last cached — which in practice means a junk position with
    // STATUS_GBAS_FIX glued on if the prior epoch was Fixed.
    if (ubx_hppos_llh_msg->invalid_lat || ubx_hppos_llh_msg->invalid_lon ||
      ubx_hppos_llh_msg->invalid_lat_hp || ubx_hppos_llh_msg->invalid_lon_hp)
    {
      RCLCPP_DEBUG(
        this->get_logger(), "dropping HPPOSLLH with invalid lat/lon flags (iTOW %u)",
        ubx_hppos_llh_msg->itow);
      return;
    }

    sensor_msgs::msg::NavSatFix nav_sat_fix_msg;
    nav_sat_fix_msg.header = ubx_hppos_llh_msg->header;

    // Extract the LLH and high-precision components
    double lat = ubx_hppos_llh_msg->lat * 1e-7 + ubx_hppos_llh_msg->lat_hp * 1e-9;
    double lon = ubx_hppos_llh_msg->lon * 1e-7 + ubx_hppos_llh_msg->lon_hp * 1e-9;
    double alt = ubx_hppos_llh_msg->height * 1e-3 + ubx_hppos_llh_msg->height_hp * 1e-4;
    nav_sat_fix_msg.latitude = lat;
    nav_sat_fix_msg.longitude = lon;
    nav_sat_fix_msg.altitude = alt;

    // h_acc / v_acc are uint32 in units of 0.1 mm (per UBX-NAV-HPPOSLLH spec).
    // Always present, always per-epoch — these are the receiver's own
    // estimate of the current solution accuracy and don't suffer the
    // bandwidth-induced drop of NAV-STATUS / NAV-COV.
    double h_acc_m = ubx_hppos_llh_msg->h_acc * 1e-4;
    double v_acc_m = ubx_hppos_llh_msg->v_acc * 1e-4;

    // Status. Prefer the receiver's own NAV-STATUS classification (which
    // distinguishes RTK Fixed / Float / DGPS / autonomous) when its iTOW
    // matches this epoch. When NAV-STATUS was dropped, classify by h_acc as
    // a robust fallback: RTK Fixed solutions report h_acc in the 3-15 mm
    // range, Float / DGPS in tens of cm, autonomous in metres.
    if (have_cached_status_ && cached_status_itow_ == ubx_hppos_llh_msg->itow) {
      nav_sat_fix_msg.status = cached_nav_sat_stat_;
    } else {
      sensor_msgs::msg::NavSatStatus status;
      status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      if (h_acc_m < 0.020) {
        status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      } else if (h_acc_m < 0.500) {
        status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      } else {
        status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      }
      nav_sat_fix_msg.status = status;
    }

    // Position covariance. Prefer the full NAV-COV NED-rotated 3x3 when its
    // iTOW matches this epoch; otherwise synthesise a diagonal from h_acc /
    // v_acc. This keeps the covariance attached to /fix self-consistent with
    // the position even when the receiver drops NAV-COV (which happens for
    // ~80 % of epochs on a ZED-F9P at 7 Hz with NAV-SAT/NAV-SIG enabled).
    if (have_cached_cov_ && cached_cov_itow_ == ubx_hppos_llh_msg->itow) {
      for (size_t i = 0; i < cached_enu_pos_cov_.size(); i++) {
        nav_sat_fix_msg.position_covariance[i] = cached_enu_pos_cov_[i];
      }
      nav_sat_fix_msg.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
    } else {
      double sigma_h_sq = h_acc_m * h_acc_m;
      double sigma_v_sq = v_acc_m * v_acc_m;
      // Row-major ENU diagonal (xx, yy, zz at [0], [4], [8]); off-diagonals 0
      nav_sat_fix_msg.position_covariance = {
        sigma_h_sq, 0.0, 0.0,
        0.0, sigma_h_sq, 0.0,
        0.0, 0.0, sigma_v_sq,
      };
      nav_sat_fix_msg.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    }

    nav_sat_fix_pub_->publish(nav_sat_fix_msg);

    RCLCPP_DEBUG(
      this->get_logger(), "Published NavSatFix with lat %4f lon %4f alt %4f", lat, lon, alt);
  }

  UBLOX_NAV_SAT_FIX_HP_NODE_LOCAL
  void nav_cov_callback(const ublox_ubx_msgs::msg::UBXNavCov::SharedPtr ubx_cov_msg)
  {
    // 6 position covariance values available in UBX-NAV-COV matrix
    // Matrix is symmetrix, so only upper triangular values are shown
    // pos_cov_nn
    // pos_cov_ne
    // pos_cov_nd
    // pos_cov_ee
    // pos_cov_ed
    // pos_cov_dd

    // In matrix notation, the values in NED coordinate system are
    // C_NED = | Pnn Pne Pnd |
    //         | Pne Pee Ped |
    //         | Pnd Ped Pdd |

    // After transformation into ENU coordinate system, the matrix becomes
    // C_ENU = | Pee  Pne -Ped |
    //         | Pne  Pnn -Pnd |
    //         |-Ped -Pnd  Pdd |

    // Tranform the covariance matrix from NED to ENU format in row-major order
    static_assert(POS_COV_ARR_SIZE == 9, "size of cached_enu_pos_cov_ must be 9");
    cached_enu_pos_cov_[0] = ubx_cov_msg->pos_cov_ee;
    cached_enu_pos_cov_[1] = ubx_cov_msg->pos_cov_ne;
    cached_enu_pos_cov_[2] = -ubx_cov_msg->pos_cov_ed;
    cached_enu_pos_cov_[3] = ubx_cov_msg->pos_cov_ne;
    cached_enu_pos_cov_[4] = ubx_cov_msg->pos_cov_nn;
    cached_enu_pos_cov_[5] = -ubx_cov_msg->pos_cov_nd;
    cached_enu_pos_cov_[6] = -ubx_cov_msg->pos_cov_ed;
    cached_enu_pos_cov_[7] = -ubx_cov_msg->pos_cov_nd;
    cached_enu_pos_cov_[8] = ubx_cov_msg->pos_cov_dd;

    cached_cov_itow_ = ubx_cov_msg->itow;
    have_cached_cov_ = true;
  }

  UBLOX_NAV_SAT_FIX_HP_NODE_LOCAL
  void nav_sta_callback(const ublox_ubx_msgs::msg::UBXNavStatus::SharedPtr ubx_sta_msg)
  {
    // UBX NAV STATUS values do not map very cleanly to ROS2 sensor_msgs/msg/NavSatStatus values.
    // Do the best we can to indicate whether we have GPS fix or not
    switch (ubx_sta_msg->gps_fix.fix_type) {
      case ublox_ubx_msgs::msg::GpsFix::GPS_NO_FIX:
      case ublox_ubx_msgs::msg::GpsFix::GPS_TIME_ONLY:
      case ublox_ubx_msgs::msg::GpsFix::GPS_DEAD_RECKONING_ONLY:
        cached_nav_sat_stat_.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        break;
      case ublox_ubx_msgs::msg::GpsFix::GPS_FIX_2D:
      case ublox_ubx_msgs::msg::GpsFix::GPS_FIX_3D:
      case ublox_ubx_msgs::msg::GpsFix::GPS_PLUS_DEAD_RECKONING:
        if (ubx_sta_msg->carr_soln.status ==
          ublox_ubx_msgs::msg::CarrSoln::CARRIER_SOLUTION_PHASE_WITH_FIXED_AMBIGUITIES)
        {
          cached_nav_sat_stat_.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        } else if (ubx_sta_msg->diff_soln) {  // diff corrections were applied
          cached_nav_sat_stat_.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        } else {
          cached_nav_sat_stat_.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        }
        break;
      default:
        cached_nav_sat_stat_.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        break;
    }

    // Service values - derive from UBX-NAV-SAT gnssId field?
    // In their absence, use arrogant default assumption of GPS
    cached_nav_sat_stat_.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    cached_status_itow_ = ubx_sta_msg->itow;
    have_cached_status_ = true;
  }
};

}  // namespace ublox_nav_sat_fix_hp

RCLCPP_COMPONENTS_REGISTER_NODE(ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode)
