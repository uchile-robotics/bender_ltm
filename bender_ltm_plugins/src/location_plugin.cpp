#include <bender_ltm_plugins/location_plugin.h>
#include <ltm/util/parameter_server_wrapper.h>

namespace bender_ltm_plugins
{
    void LocationPlugin::initialize(const std::string& param_ns) {
        _log_prefix = "[LTM][Location]: ";
        ROS_DEBUG_STREAM(_log_prefix << "Initializing with ns: " << param_ns);

        _first_pose_msg = false;

        // parameters
        ltm::util::ParameterServerWrapper psw("~");
        psw.getParameter(param_ns + "map_name", _map_name, "amtc");
        psw.getParameter(param_ns + "pose_topic", _pose_topic, "/bender/nav/amcl_pose");
        psw.getParameter(param_ns + "location_service", _location_service, "/bender/knowledge/map_analyzer/check_point_inside_map");

        ros::NodeHandle priv("~");
        _pose_sub = priv.subscribe(_pose_topic, 1, &LocationPlugin::pose_callback, this);
        _location_client = priv.serviceClient<uchile_srvs::ValidPoint>(_location_service);

        _initialized = false;
        reinitialize();
    }

    bool LocationPlugin::reinitialize() {
        if (!_location_client.waitForExistence(ros::Duration(2.0))) {
            ROS_WARN_STREAM(_log_prefix << "Couldn't find service " << _location_client.getService() << ".");
            return false;
        }
        ROS_INFO_STREAM(_log_prefix << "initialized.");
        _initialized = true;
        return true;
    }

    void LocationPlugin::reset() {
        ROS_DEBUG_STREAM(_log_prefix << "resetting location plugin.");
    }

    void LocationPlugin::register_episode(uint32_t uid) {
        ROS_DEBUG_STREAM(_log_prefix << "registering episode " << uid);
        if (!_initialized) reinitialize();
    }

    void LocationPlugin::unregister_episode(uint32_t uid) {
        ROS_DEBUG_STREAM(_log_prefix << "unregistering episode " << uid);
    }

    void LocationPlugin::pose_callback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
        _robot_position = msg.pose.pose.position;
        _robot_position_frame_id = msg.header.frame_id;
        _first_pose_msg = true;
    }

    void LocationPlugin::collect(uint32_t uid, ltm::Where& where) {
        ROS_DEBUG_STREAM(_log_prefix << "collecting episode INIT" << uid);
        if (!_initialized && !reinitialize()) return;
        if (!_first_pose_msg) {
            ROS_WARN_STREAM(_log_prefix << "plugin is working, but no pose messages have been received yet from topic: " + _pose_topic + ".");
            return;
        }

        where.map_name = _map_name;
        where.position = _robot_position;
        where.frame_id = _robot_position_frame_id;

        // CHECK LOCATION STRING
        uchile_srvs::ValidPoint srv;
        srv.request.point = _robot_position;
        srv.request.frame_id = _robot_position_frame_id;
        if (_location_client.call(srv)) {

            bool point_is_inside_map = srv.response.is_valid;
            std::vector<std::string> locations = srv.response.data;
            if (!point_is_inside_map) {
                where.area = "outside_map";
                where.location = "unknown";
            } else {
                where.area = "inside_map";
                where.location = (locations.size() > 0)  ? locations[0] : "unknown";
            }

        } else {
            ROS_WARN_STREAM(_log_prefix << "service call failed.(" << _location_client.getService() << ").");
        }
    }

}
