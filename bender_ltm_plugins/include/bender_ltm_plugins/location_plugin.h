#ifndef BENDER_LTM_PLUGINS_LOCATION_PLUGIN_H_
#define BENDER_LTM_PLUGINS_LOCATION_PLUGIN_H_

#include <ros/ros.h>
#include <ltm/plugin/location_base.h>
#include <uchile_srvs/ValidPoint.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>

namespace bender_ltm_plugins
{
    class LocationPlugin : public ltm::plugin::LocationBase
    {
    public:
        LocationPlugin(){}

        void initialize(const std::string& param_ns);

        void register_episode(uint32_t uid);

        void unregister_episode(uint32_t uid);

        void collect(uint32_t uid, ltm::Where& msg);

        void reset();

    private:
        bool _initialized;
        std::string _log_prefix;

        // Collect position through geometry_msgs::PoseWithCovarianceStamped msgs.
        std::string _pose_topic;
        ros::Subscriber _pose_sub;
        geometry_msgs::Point _robot_position;
        std::string _robot_position_frame_id;
        bool _first_pose_msg;

        // Collect location names through a ServiceClient using uchile_srvs::ValidPoint srvs.
        std::string _location_service;
        ros::ServiceClient _location_client;

        void pose_callback(const geometry_msgs::PoseWithCovarianceStamped &msg);
        bool reinitialize();
    };

};
#endif // BENDER_LTM_PLUGINS_LOCATION_PLUGIN_H_
