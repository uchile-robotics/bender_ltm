#include <bender_ltm_plugins/emotion_plugin.h>

namespace bender_ltm_plugins
{
    void EmotionPlugin::initialize(const std::string &param_ns) {
        _log_prefix = "[LTM][Emotion]: ";
        ROS_DEBUG_STREAM(_log_prefix << "Initializing with ns: " << param_ns);
    
        _initialized = false;
        reinitialize();
    }

    void EmotionPlugin::register_episode(uint32_t uid) {
        ROS_DEBUG_STREAM(_log_prefix << "registering episode " << uid);
        if (!_initialized) reinitialize();
    }

    void EmotionPlugin::unregister_episode(uint32_t uid) {
        ROS_DEBUG_STREAM(_log_prefix << "unregistering episode " << uid);
    }

    void EmotionPlugin::collect(uint32_t uid, ltm::EmotionalRelevance &msg) {
        ROS_DEBUG_STREAM(_log_prefix << "collecting episode " << uid);
        if (!_initialized && !reinitialize()) return;
    }


    bool EmotionPlugin::reinitialize() {
        ROS_INFO_STREAM(_log_prefix << "initialized.");
        _initialized = true;
        return true;
    }

    void EmotionPlugin::reset() {
        ROS_INFO_STREAM(_log_prefix << "reset.");
    }

}
