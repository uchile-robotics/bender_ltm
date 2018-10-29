#include <bender_ltm_plugins/emotion_plugin.h>

namespace bender_ltm_plugins
{
    void EmotionPlugin::initialize(const std::string &param_ns) {
        log_prefix = "[LTM][Emotion]: ";
        ROS_DEBUG_STREAM(log_prefix << "Initializing with ns: " << param_ns);
        ros::NodeHandle priv("~");
        client = priv.serviceClient<ltm::GetEpisodes>("/robot/fake/get_emotion");

        initialized = false;
        reinitialize();
    }

    void EmotionPlugin::register_episode(uint32_t uid) {
        ROS_DEBUG_STREAM(log_prefix << "registering episode " << uid);
        registry[uid] = ros::Time::now();

        if (!initialized) reinitialize();
    }

    void EmotionPlugin::unregister_episode(uint32_t uid) {
        ROS_DEBUG_STREAM(log_prefix << "unregistering episode " << uid);
        registry.erase(uid);
    }

    void EmotionPlugin::collect(uint32_t uid, ltm::EmotionalRelevance &msg) {
        ROS_DEBUG_STREAM(log_prefix << "collecting episode " << uid);
        if (!initialized && !reinitialize()) {
            ROS_WARN_STREAM(log_prefix << "Required service is not present (" << client.getService() << ").");
            return;
        }
        ltm::GetEpisodes srv;
        if (client.call(srv)) {
            msg = srv.response.episodes[0].relevance.emotional;
        } else {
            ROS_WARN_STREAM(log_prefix << "service call failed.(" << client.getService() << ").");
        }
    }


    bool EmotionPlugin::reinitialize() {
        if (!client.waitForExistence(ros::Duration(2.0))) {
            ROS_WARN_STREAM(log_prefix << "Couldn't find service " << client.getService() << ".");
            return false;
        }
        ROS_INFO_STREAM(log_prefix << "initialized.");
        initialized = true;
        return true;
    }

    void EmotionPlugin::reset() {

    }

}
