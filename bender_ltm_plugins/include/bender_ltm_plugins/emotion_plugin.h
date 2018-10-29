#ifndef BENDER_LTM_PLUGINS_EMOTION_PLUGIN_H_
#define BENDER_LTM_PLUGINS_EMOTION_PLUGIN_H_

#include <ros/ros.h>
#include <ltm/emotion_base.h>
#include <ltm/GetEpisodes.h>

namespace bender_ltm_plugins
{
    class EmotionPlugin : public ltm::plugin::EmotionBase
    {
    public:
        EmotionPlugin() {}

        void initialize(const std::string &param_ns);

        void register_episode(uint32_t uid);

        void unregister_episode(uint32_t uid);

        void collect(uint32_t uid, ltm::EmotionalRelevance &msg);

        void reset();

    private:
        ros::ServiceClient client;
        std::map<uint32_t, ros::Time> registry;
        bool initialized;
        std::string log_prefix;
        bool reinitialize();
    };

};
#endif // BENDER_LTM_PLUGINS_EMOTION_PLUGIN_H_
