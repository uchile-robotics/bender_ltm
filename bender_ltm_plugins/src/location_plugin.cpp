#include <bender_ltm_plugins/location_plugin.h>

namespace bender_ltm_plugins
{
    void LocationPlugin::initialize(const std::string& param_ns) {
        log_prefix = "[LTM][Location]: ";
        ROS_DEBUG_STREAM(log_prefix << "Initializing with ns: " << param_ns);
        ros::NodeHandle priv("~");
        client = priv.serviceClient<ltm::GetEpisodes>("/robot/fake/get_location");

        initialized = false;
        reinitialize();
    }

    void LocationPlugin::register_episode(uint32_t uid) {
        ROS_DEBUG_STREAM(log_prefix << "registering episode " << uid);
        registry[uid] = ros::Time::now();

        if (!initialized) reinitialize();
    }

    void LocationPlugin::unregister_episode(uint32_t uid) {
        ROS_DEBUG_STREAM(log_prefix << "unregistering episode " << uid);
        registry.erase(uid);
    }

    void LocationPlugin::collect(uint32_t uid, ltm::Where& msg) {
        ROS_DEBUG_STREAM(log_prefix << "collecting episode " << uid);
        if (!initialized && !reinitialize()) {
            ROS_WARN_STREAM(log_prefix << "Required service is not present (" << client.getService() << ").");
            return;
        }
        ltm::GetEpisodes srv;
        if (client.call(srv)) {
            msg = srv.response.episodes[0].where;
        } else {
            ROS_WARN_STREAM(log_prefix << "service call failed.(" << client.getService() << ").");
        }
    }

    bool LocationPlugin::reinitialize() {
        if (!client.waitForExistence(ros::Duration(2.0))) {
            ROS_WARN_STREAM(log_prefix << "Couldn't find service " << client.getService() << ".");
            return false;
        }
        ROS_INFO_STREAM(log_prefix << "initialized.");
        initialized = true;
        return true;
    }

    void LocationPlugin::reset() {

    }
}
