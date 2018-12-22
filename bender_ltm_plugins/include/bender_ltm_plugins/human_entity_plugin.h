#ifndef BENDER_LTM_PLUGINS_HUMAN_ENTITY_PLUGIN_H_
#define BENDER_LTM_PLUGINS_HUMAN_ENTITY_PLUGIN_H_

#include <ros/ros.h>
#include <ltm/util/util.h>
#include <ltm/plugin/entity_base.h>
#include <ltm/plugin/entity_default.h>
#include <bender_ltm_plugins/HumanEntity.h>
#include <bender_ltm_plugins/HumanEntitySrv.h>
#include <ltm/QueryServer.h>
#include <std_srvs/Empty.h>
#include <ltm/plugin/entity_util.h>

namespace bender_ltm_plugins
{
    struct RegisterItem {
        ros::Time timestamp;
        uint32_t log_uid;
        uint32_t entity_uid;

        RegisterItem(ros::Time timestamp, uint32_t log_uid, uint32_t entity_uid) {
            this->timestamp = timestamp;
            this->log_uid = log_uid;
            this->entity_uid = entity_uid;
        }

    };
    struct RegisterItemComp {
        bool operator() (const RegisterItem& lhs, const RegisterItem& rhs) const
        {return lhs.timestamp < rhs.timestamp;}
    };

    class HumanEntityPlugin :
            public ltm::plugin::EntityBase,
            public ltm::plugin::EntityDefault<bender_ltm_plugins::HumanEntity, bender_ltm_plugins::HumanEntitySrv>
    {
    private:
        // Entity Types
        typedef bender_ltm_plugins::HumanEntity EntityMsg;
        typedef ltm_db::MessageWithMetadata<EntityMsg> EntityWithMetadata;
        typedef boost::shared_ptr<const EntityWithMetadata> EntityWithMetadataPtr;

        // Log Message Types
        typedef ltm::EntityLog LogType;
        typedef ltm_db::MessageWithMetadata<LogType> LogWithMetadata;
        typedef boost::shared_ptr<const LogWithMetadata> LogWithMetadataPtr;

        // ROS API
        std::string _log_prefix;
        std::string _stm_topic;
        ros::Subscriber _sub;

        // timestamp registry (keep sorted by timestamp)
        typedef std::multiset<RegisterItem, RegisterItemComp> Registry;
        Registry _registry;

    public:
        HumanEntityPlugin(){}
        ~HumanEntityPlugin();

    private:
        void callback(const EntityMsg &msg);
        uint32_t lookup_uid(std::string name);
        void build_null(EntityMsg &entity);
        void fill_field_names(std::set<std::string> &names);

    public:
        // EntityBase Methods
        std::string get_type();
        void initialize(const std::string &param_ns, DBConnectionPtr db_ptr, std::string db_name);
        void register_episode(uint32_t uid);
        void unregister_episode(uint32_t uid);
        void collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end);
        void query(const std::string &json, ltm::QueryServer::Response &res, bool trail);
        void drop_db();
        void reset(const std::string &db_name);
        void append_status(std::stringstream &status);

        // EntityCollection Methods
        MetadataPtr make_metadata(const EntityMsg &entity);
        void retrace(EntityMsg &entity, const std::vector<uint32_t> &logs);
        void update(const EntityMsg &msg);
        void copy_field(const std::string& name, EntityWithMetadataPtr &in, EntityMsg &out);
    };


};
#endif // BENDER_LTM_PLUGINS_HUMAN_ENTITY_PLUGIN_H_
