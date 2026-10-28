#include "los4_diagnostics.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Geometry>
#include <mutex>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <thread>
#include <optional>
#include <stdexcept>

using Telemetry = mavsdk::Telemetry;
struct Cache {
    std::mutex mutex;
    uint8_t selected_system=0;
    std::optional<mavlink_heartbeat_t> heartbeat;
    std::optional<mavlink_rc_channels_t> rc;
    std::optional<std::vector<float>> los4_config, los4_state, los4_environment;
    std::optional<std::pair<uint8_t,uint8_t>> los4_source;
    std::optional<Telemetry::Position> gps;
    std::optional<Telemetry::PositionVelocityNed> local;
    std::optional<Telemetry::Quaternion> attitude;
    std::optional<Telemetry::AngularVelocityBody> rates;
    std::optional<Telemetry::ActuatorControlTarget> controls;
    std::optional<Telemetry::ActuatorOutputStatus> outputs;
};

class TelemetryBridge : public rclcpp::Node {
public:
    TelemetryBridge() : Node("mavsdk_telemetry_bridge"), cache_(std::make_shared<Cache>()) {
        const auto url = declare_parameter<std::string>("connection_url", "udp://:14540");
        const double rate = declare_parameter<double>("telemetry_rate_hz", 20.0);
        const double timeout = declare_parameter<double>("discovery_timeout_s", 30.0);
        const int system_id = declare_parameter<int>("system_id", 0);
        if (!std::isfinite(rate) || rate <= 0 || !std::isfinite(timeout) || timeout <= 0 ||
            system_id < 0 || system_id > 255) throw std::invalid_argument("Invalid rate, timeout or system ID");
        kill_channel_ = declare_parameter<int>("kill_channel", 6);
        kill_threshold_ = declare_parameter<int>("kill_threshold", 1700);
        kill_active_high_ = declare_parameter<bool>("kill_active_high", true);
        if(kill_channel_<1 || kill_channel_>18 || kill_threshold_<800 || kill_threshold_>2200)
            throw std::invalid_argument("Invalid kill switch configuration");
        const auto qos = rclcpp::SensorDataQoS();
        mode_pub_ = create_publisher<std_msgs::msg::String>("telemetry/flight_mode", qos);
        armed_pub_ = create_publisher<std_msgs::msg::Bool>("telemetry/armed", qos);
        kill_pub_ = create_publisher<std_msgs::msg::Bool>("telemetry/kill_active", qos);
        rc_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("telemetry/rc_channels", qos);
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("telemetry/pose", qos);
        velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("telemetry/velocity", qos);
        attitude_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("telemetry/attitude", qos);
        rates_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("telemetry/body_rates", qos);
        gps_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("telemetry/gps", qos);
        altitude_pub_ = create_publisher<std_msgs::msg::Float32>("telemetry/relative_altitude", qos);
        controls_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("telemetry/actuator_controls", qos);
        group_pub_ = create_publisher<std_msgs::msg::Int32>("telemetry/actuator_control_group", qos);
        outputs_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("telemetry/actuator_outputs", qos);
        active_pub_ = create_publisher<std_msgs::msg::UInt32>("telemetry/actuator_active", qos);
        los4_config_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
            "telemetry/los4/config", rclcpp::QoS(1).transient_local().reliable());
        los4_state_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("telemetry/los4/state", qos);
        environment_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("telemetry/los4/environment", qos);
        airspeed_pub_ = create_publisher<std_msgs::msg::Float32>("telemetry/airspeed", qos);
        legacy_tas_pub_ = create_publisher<std_msgs::msg::Float32>("telemetry/true_airspeed", qos);
        calibrated_airspeed_pub_ = create_publisher<std_msgs::msg::Float32>("telemetry/calibrated_airspeed", qos);
        wind_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("telemetry/wind", qos);
        wind_enabled_pub_ = create_publisher<std_msgs::msg::Bool>("telemetry/wind_enabled", qos);
        airspeed_valid_pub_ = create_publisher<std_msgs::msg::Bool>("telemetry/airspeed_valid", qos);
        wind_valid_pub_ = create_publisher<std_msgs::msg::Bool>("telemetry/wind_valid", qos);
        const auto cache = cache_;
        sdk_ = std::make_unique<mavsdk::Mavsdk>(
            mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::GroundStation});
        sdk_->intercept_incoming_messages_async([cache](mavlink_message_t& message) {
            if(message.msgid==MAVLINK_MSG_ID_HEARTBEAT || message.msgid==MAVLINK_MSG_ID_RC_CHANNELS) {
                std::lock_guard<std::mutex> lock(cache->mutex);
                if(message.sysid!=cache->selected_system || message.compid!=MAV_COMP_ID_AUTOPILOT1)return true;
                if(message.msgid==MAVLINK_MSG_ID_HEARTBEAT) {
                    mavlink_heartbeat_t value{};mavlink_msg_heartbeat_decode(&message,&value);cache->heartbeat=value;
                } else {
                    mavlink_rc_channels_t value{};mavlink_msg_rc_channels_decode(&message,&value);cache->rc=value;
                }
                return true;
            }
            if(message.msgid!=MAVLINK_MSG_ID_TUNNEL)return true;
            mavlink_tunnel_t tunnel{};mavlink_msg_tunnel_decode(&message,&tunnel);
            if(tunnel.payload_type!=los4_wire::payload_type)return true;
            auto values=los4_wire::decode(tunnel.payload,tunnel.payload_length);
            if(values.empty())return true;
            std::lock_guard<std::mutex> lock(cache->mutex);
            const auto source=std::make_pair(message.sysid,message.compid);
            if(cache->los4_source && *cache->los4_source!=source)return true;
            cache->los4_source=source;
            if(values[1]==1)cache->los4_config=std::move(values);
            else if(values[1]==2)cache->los4_state=std::move(values);
            else cache->los4_environment=std::move(values);
            return true;
        });
        if (sdk_->add_any_connection(url) != mavsdk::ConnectionResult::Success)
            throw std::runtime_error("Cannot open MAVLink endpoint: " + url);
        RCLCPP_INFO(get_logger(), "Waiting for MAVLink on %s", url.c_str());
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline && !system_) {
            for (const auto& candidate : sdk_->systems()) {
                if (candidate->is_connected() && candidate->has_autopilot() &&
                    (!system_id || candidate->get_system_id() == system_id)) {
                    system_ = candidate;
                    break;
                }
            }
            if (!system_) std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!system_) throw std::runtime_error("No requested autopilot within discovery timeout");
        {std::lock_guard<std::mutex> lock(cache_->mutex);cache_->selected_system=system_->get_system_id();}
        telemetry_ = std::make_unique<Telemetry>(system_);
        telemetry_->subscribe_position([cache](auto value) {
            std::lock_guard<std::mutex> lock(cache->mutex); cache->gps = value;
        });
        telemetry_->subscribe_position_velocity_ned([cache](auto value) {
            std::lock_guard<std::mutex> lock(cache->mutex); cache->local = value;
        });
        telemetry_->subscribe_attitude_quaternion([cache](auto value) {
            std::lock_guard<std::mutex> lock(cache->mutex); cache->attitude = value;
        });
        telemetry_->subscribe_attitude_angular_velocity_body([cache](auto value) {
            std::lock_guard<std::mutex> lock(cache->mutex); cache->rates = value;
        });
        telemetry_->subscribe_actuator_control_target([cache](auto value) {
            std::lock_guard<std::mutex> lock(cache->mutex); cache->controls = value;
        });
        telemetry_->subscribe_actuator_output_status([cache](auto value) {
            std::lock_guard<std::mutex> lock(cache->mutex); cache->outputs = value;
        });
        telemetry_->set_rate_rc_status(10.0);
        // Requests may be unsupported; streams that do arrive remain usable.
        for (const auto result : {telemetry_->set_rate_position(rate),
                                 telemetry_->set_rate_position_velocity_ned(rate),
                                 telemetry_->set_rate_attitude_quaternion(rate),
                                 telemetry_->set_rate_actuator_control_target(rate),
                                 telemetry_->set_rate_actuator_output_status(rate)}) {
            if (result != Telemetry::Result::Success)
                RCLCPP_WARN_STREAM(get_logger(), "Telemetry rate request failed: " << result);
        }
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { publish(); });
        RCLCPP_INFO(get_logger(), "Connected; publishing telemetry as ROS 2 topics");
    }
private:
    void publish() {
        // Consume new samples once. Missing streams never become fabricated zeros.
        std::optional<Telemetry::Position> gps;
        std::optional<Telemetry::PositionVelocityNed> local;
        std::optional<Telemetry::Quaternion> attitude;
        std::optional<Telemetry::AngularVelocityBody> rates;
        std::optional<Telemetry::ActuatorControlTarget> controls;
        std::optional<Telemetry::ActuatorOutputStatus> outputs;
        {
            std::lock_guard<std::mutex> lock(cache_->mutex);
            gps.swap(cache_->gps); local.swap(cache_->local); rates.swap(cache_->rates);
            controls.swap(cache_->controls); outputs.swap(cache_->outputs);
            attitude = cache_->attitude;
        }
        std::optional<std::vector<float>> config, state, environment;
        {std::lock_guard<std::mutex> lock(cache_->mutex);
            config.swap(cache_->los4_config);state.swap(cache_->los4_state);environment.swap(cache_->los4_environment);}
        if(config) {std_msgs::msg::Float32MultiArray msg;msg.data=*config;los4_config_pub_->publish(msg);}
        if(state) {std_msgs::msg::Float32MultiArray msg;msg.data=*state;los4_state_pub_->publish(msg);}
        if(environment) {
            const auto& v=*environment;
            std_msgs::msg::Float32MultiArray raw;raw.data=v;environment_pub_->publish(raw);
            std_msgs::msg::Bool flag;flag.data=v[2]==1;wind_enabled_pub_->publish(flag);
            flag.data=v[9]==1;airspeed_valid_pub_->publish(flag);
            flag.data=v[10]==1;wind_valid_pub_->publish(flag);
            std_msgs::msg::Float32 air;
            if(v[0]==1) {air.data=v[3];legacy_tas_pub_->publish(air);} // Old recordings only.
            air.data=v[9]==1?v[4]:NAN;airspeed_pub_->publish(air);
            air.data=v[4];calibrated_airspeed_pub_->publish(air);
            geometry_msgs::msg::Vector3Stamped wind;
            wind.header.stamp=now();wind.header.frame_id="map_ned";
            wind.vector.x=v[10]==1?v[5]:NAN;wind.vector.y=v[10]==1?v[6]:NAN;
            wind.vector.z=NAN; // PX4 provides a horizontal estimate; vertical wind unknown.
            wind_pub_->publish(wind);
        }
        std::optional<mavlink_heartbeat_t> heartbeat;
        std::optional<mavlink_rc_channels_t> rc;
        {std::lock_guard<std::mutex> lock(cache_->mutex);heartbeat.swap(cache_->heartbeat);rc.swap(cache_->rc);}
        if(heartbeat) {
            const unsigned main=(heartbeat->custom_mode>>16)&0xff;
            const unsigned sub=(heartbeat->custom_mode>>24)&0xff;
            std::string mode;
            switch(main) {
            case 1:mode="MANUAL";break;case 2:mode="ALTITUDE";break;case 3:mode="POSITION";break;
            case 4:mode="AUTO ("+std::to_string(sub)+")";break;
            case 5:mode="ACRO";break;case 6:mode="OFFBOARD";break;case 7:mode="STABILIZED";break;
            case 8:mode="RATTITUDE";break;default:mode="UNKNOWN ("+std::to_string(main)+")";break;
            }
            std_msgs::msg::String msg;msg.data=mode;mode_pub_->publish(msg);
            std_msgs::msg::Bool armed;armed.data=(heartbeat->base_mode & MAV_MODE_FLAG_SAFETY_ARMED)!=0;
            armed_pub_->publish(armed);
        }
        if(rc) {
            const uint16_t channels[]={rc->chan1_raw,rc->chan2_raw,rc->chan3_raw,rc->chan4_raw,rc->chan5_raw,
                rc->chan6_raw,rc->chan7_raw,rc->chan8_raw,rc->chan9_raw,rc->chan10_raw,rc->chan11_raw,rc->chan12_raw,
                rc->chan13_raw,rc->chan14_raw,rc->chan15_raw,rc->chan16_raw,rc->chan17_raw,rc->chan18_raw};
            std_msgs::msg::Float32MultiArray raw;
            for(unsigned i=0;i<std::min<unsigned>(18,rc->chancount);++i)raw.data.push_back(channels[i]);
            rc_pub_->publish(raw);
            const auto pwm=channels[kill_channel_-1];
            if(kill_channel_<=rc->chancount && pwm>=800 && pwm<=2200 && rc->rssi!=0) {
                std_msgs::msg::Bool kill;
                kill.data=kill_active_high_ ? pwm>=kill_threshold_ : pwm<=kill_threshold_;
                kill_pub_->publish(kill);
            }
        }
        const auto stamp = now();
        if (gps) {
            sensor_msgs::msg::NavSatFix msg;
            msg.header.stamp = stamp; msg.header.frame_id = "gps";
            msg.latitude = gps->latitude_deg; msg.longitude = gps->longitude_deg;
            msg.altitude = gps->absolute_altitude_m;
            // MAVSDK Position has no fix-quality/covariance; do not infer it here.
            msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            gps_pub_->publish(msg);
            std_msgs::msg::Float32 relative; relative.data = gps->relative_altitude_m;
            altitude_pub_->publish(relative);
        }
        Eigen::Quaterniond q;
        bool have_attitude = false;
        if (attitude) {
            q = Eigen::Quaterniond(attitude->w, attitude->x, attitude->y, attitude->z);
            have_attitude = q.coeffs().allFinite() && q.norm() > 1e-6;
            if (have_attitude) {
                q.normalize();
                if (attitude->timestamp_us != last_attitude_timestamp_) {
                    const Eigen::Matrix3d R = q.toRotationMatrix();
                    geometry_msgs::msg::Vector3Stamped msg;
                    msg.header.stamp = stamp; msg.header.frame_id = "body_frd";
                    msg.vector.x = std::atan2(R(2,1), R(2,2));
                    msg.vector.y = std::atan2(-R(2,0), std::hypot(R(2,1), R(2,2)));
                    msg.vector.z = std::atan2(R(1,0), R(0,0));
                    attitude_pub_->publish(msg);
                    last_attitude_timestamp_ = attitude->timestamp_us;
                }
            }
        }
        if (local) {
            const auto& p = local->position; const auto& v = local->velocity;
            if (std::isfinite(v.north_m_s) && std::isfinite(v.east_m_s) && std::isfinite(v.down_m_s)) {
                geometry_msgs::msg::TwistStamped msg;
                msg.header.stamp = stamp; msg.header.frame_id = "map_ned";
                msg.twist.linear.x = v.north_m_s; msg.twist.linear.y = v.east_m_s;
                msg.twist.linear.z = v.down_m_s; velocity_pub_->publish(msg);
            }
            if (have_attitude && std::isfinite(p.north_m) && std::isfinite(p.east_m) && std::isfinite(p.down_m)) {
                geometry_msgs::msg::PoseStamped msg;
                msg.header.stamp = stamp; msg.header.frame_id = "map_ned";
                msg.pose.position.x = p.north_m; msg.pose.position.y = p.east_m; msg.pose.position.z = p.down_m;
                // Native MAVSDK quaternion: body FRD orientation in world NED.
                msg.pose.orientation.w = q.w(); msg.pose.orientation.x = q.x();
                msg.pose.orientation.y = q.y(); msg.pose.orientation.z = q.z();
                pose_pub_->publish(msg);
            }
        }
        if (rates) {
            geometry_msgs::msg::Vector3Stamped msg;
            msg.header.stamp = stamp; msg.header.frame_id = "body_frd";
            msg.vector.x = rates->roll_rad_s; msg.vector.y = rates->pitch_rad_s;
            msg.vector.z = rates->yaw_rad_s; rates_pub_->publish(msg);
        }
        if (controls) {
            std_msgs::msg::Float32MultiArray msg; msg.data = controls->controls;
            controls_pub_->publish(msg);
            std_msgs::msg::Int32 group; group.data = controls->group; group_pub_->publish(group);
        }
        if (outputs) {
            std_msgs::msg::Float32MultiArray msg; msg.data = outputs->actuator; outputs_pub_->publish(msg);
            std_msgs::msg::UInt32 active; active.data = outputs->active; active_pub_->publish(active);
        }
    }
    int kill_channel_=6,kill_threshold_=1700;
    bool kill_active_high_=true;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_,kill_pub_,wind_enabled_pub_,airspeed_valid_pub_,wind_valid_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr rc_pub_;
    std::shared_ptr<Cache> cache_;
    std::unique_ptr<mavsdk::Mavsdk> sdk_;
    std::shared_ptr<mavsdk::System> system_;
    std::unique_ptr<Telemetry> telemetry_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t last_attitude_timestamp_{UINT64_MAX};
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_pub_, rates_pub_,wind_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_pub_,airspeed_pub_,calibrated_airspeed_pub_,legacy_tas_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr controls_pub_, outputs_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr active_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr los4_config_pub_,los4_state_pub_,environment_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr group_pub_;
};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int result = 0;
    try { rclcpp::spin(std::make_shared<TelemetryBridge>()); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; result = 1; }
    if (rclcpp::ok()) rclcpp::shutdown();
    return result;
}
