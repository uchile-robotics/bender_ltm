#include <bender_ltm_plugins/crowd_entity_plugin.h>
#include <ltm/util/parameter_server_wrapper.h>
#include <sensor_msgs/Image.h>


namespace bender_ltm_plugins
{
    using namespace ltm::plugin;

    // =================================================================================================================
    // Public API
    // =================================================================================================================

    std::string CrowdEntityPlugin::get_type() {
        return this->ltm_get_type();
    }

    void CrowdEntityPlugin::initialize(const std::string &param_ns, DBConnectionPtr db_ptr, std::string db_name) {
        _log_prefix = "[LTM][Human Entity]: ";
        ROS_DEBUG_STREAM(_log_prefix << "plugin initialized with ns: " << param_ns);

        build_null(_null_e);

        // all fields
        _field_names.insert("location");
        _field_names.insert("n_people");
        _field_names.insert("n_male");
        _field_names.insert("n_female");
        _field_names.insert("n_children");
        _field_names.insert("n_adults");
        _field_names.insert("n_elders");
        

        // parameters
        ltm::util::ParameterServerWrapper psw("~");
        psw.getParameter(param_ns + "topic", _stm_topic, "/bender/ltm/entity/crowd/update");

        ros::NodeHandle priv("~");
        _sub = priv.subscribe(_stm_topic, 1, &CrowdEntityPlugin::callback, this);

        // DB connection
        this->ltm_setup(param_ns, db_ptr, db_name);

        // init ROS interface
        this->ltm_init();
    }

    CrowdEntityPlugin::~CrowdEntityPlugin() {

    }

    void CrowdEntityPlugin::register_episode(uint32_t uid) {
        this->ltm_register_episode(uid);
        // TODO: WE CAN USE THIS TO DELETE OLD TIMESTAMPS FROM THE REGISTRY
    }

    void CrowdEntityPlugin::unregister_episode(uint32_t uid) {
        this->ltm_unregister_episode(uid);
        // TODO: WE CAN USE THIS TO DELETE OLD TIMESTAMPS FROM THE REGISTRY
    }

    void CrowdEntityPlugin::collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end) {
        ROS_WARN_STREAM(_log_prefix << "Collecting Human entities for episode " << uid << ".");

        // TODO: mutex (it is not required, because we are using a single-threaded Spinner for callbacks)

        // write entities matching initial and ending times.
        std::map<uint32_t, ltm::EntityRegister> episode_registry;
        std::map<uint32_t, ltm::EntityRegister>::iterator m_it;

        // COMPUTE EPISODE REGISTRY
        int logcnt = 0;
        Registry::const_iterator it;
        for (it = _registry.lower_bound(RegisterItem2(_start, 0, 0)); it != _registry.end(); ++it) {
            if (it->timestamp > _end) break;

            // entity is in registry
            m_it = episode_registry.find(it->entity_uid);
            if (m_it != episode_registry.end()) {
                // update register
                m_it->second.log_uids.push_back(it->log_uid);
            } else {
                // new register
                ltm::EntityRegister reg;
                reg.type = ltm_get_type();
                reg.uid = it->entity_uid;
                reg.log_uids.push_back(it->log_uid);
                episode_registry.insert(std::pair<uint32_t, ltm::EntityRegister>(it->entity_uid, reg));
            }
            logcnt++;
        }

        // SAVE IT
        ROS_WARN_STREAM(_log_prefix << "Collected (" << episode_registry.size() << ") Human entities and (" << logcnt << ") logs for episode " << uid << ".");
        for (m_it = episode_registry.begin(); m_it != episode_registry.end(); ++m_it) {
            msg.entities.push_back(m_it->second);
        }

        // unregister
        // TODO: redundant calls to (un)register methods?
        unregister_episode(uid);
    }

    void CrowdEntityPlugin::query(const std::string &json, ltm::QueryServer::Response &res, bool trail) {
        this->ltm_query(json, res, trail);
    }

    void CrowdEntityPlugin::drop_db() {
        this->reset(this->ltm_get_db_name());
        this->ltm_drop_db();
    }

    void CrowdEntityPlugin::reset(const std::string &db_name) {
        this->_registry.clear();
        this->ltm_resetup_db(db_name);
    }

    void CrowdEntityPlugin::append_status(std::stringstream &status) {
        status << this->ltm_get_status();
    }

    MetadataPtr CrowdEntityPlugin::make_metadata(const EntityMsg &entity) {
        MetadataPtr meta = this->ltm_create_metadata(entity);
        meta->append("location", entity.location);
        meta->append("n_people", entity.n_people);
        meta->append("n_male", entity.n_male);
        meta->append("n_female", entity.n_female);
        meta->append("n_children", entity.n_children);
        meta->append("n_adults", entity.n_adults);
        meta->append("n_elders", entity.n_elders);
        return meta;
    }

    // =================================================================================================================
    // Private API
    // =================================================================================================================

    void CrowdEntityPlugin::callback(const EntityMsg &msg) {
        this->update(msg);
    }

    void CrowdEntityPlugin::update(const EntityMsg& msg) {
        // KEYS
        LogType log;
        log.entity_uid = msg.meta.uid;
        log.log_uid = (uint32_t) this->ltm_reserve_log_uid();

        // WHEN
        log.timestamp = ros::Time::now();
        log.prev_log = (uint32_t) this->ltm_get_last_log_uid(log.entity_uid);
        // log.next_log = 0;

        // WHO
        this->ltm_get_registry(log.episode_uids);

        // TODO: WE CAN USE A CACHE FOR RECENT ENTITIES
        EntityMsg curr = _null_e;
        EntityWithMetadataPtr curr_with_md;
        curr.meta.uid = msg.meta.uid;
        curr.meta.log_uid = log.log_uid;
        bool uid_exists = this->ltm_get_last(msg.meta.uid, curr_with_md);
        if (uid_exists) {
            curr.location = curr_with_md->location;
            curr.n_people = curr_with_md->n_people;
            curr.n_male = curr_with_md->n_male;
            curr.n_female = curr_with_md->n_female;
            curr.n_children = curr_with_md->n_children;
            curr.n_adults = curr_with_md->n_adults;
            curr.n_elders = curr_with_md->n_elders;
        } else {
            // NEW ENTITY
            curr.meta.init_log = curr.meta.log_uid;
            curr.meta.init_stamp = log.timestamp;
        }
        curr.meta.stamp = log.timestamp;
        curr.meta.last_log = log.log_uid;
        curr.meta.last_stamp = log.timestamp;

        EntityMsg diff;
        diff.meta = curr.meta;
        entity::update_field<std::string>(log, "location", curr.location, diff.location, msg.location, _null_e.location);
        entity::update_field<uint8_t>(log, "n_people", curr.n_people, diff.n_people, msg.n_people, _null_e.n_people);
        entity::update_field<uint8_t>(log, "n_male", curr.n_male, diff.n_male, msg.n_male, _null_e.n_male);
        entity::update_field<uint8_t>(log, "n_female", curr.n_female, diff.n_female, msg.n_female, _null_e.n_female);
        entity::update_field<uint8_t>(log, "n_children", curr.n_children, diff.n_children, msg.n_children, _null_e.n_children);
        entity::update_field<uint8_t>(log, "n_adults", curr.n_adults, diff.n_adults, msg.n_adults, _null_e.n_adults);
        entity::update_field<uint8_t>(log, "n_elders", curr.n_elders, diff.n_elders, msg.n_elders, _null_e.n_elders);
        
        size_t n_added = log.new_f.size();
        size_t n_updated = log.updated_f.size();
        size_t n_removed = log.removed_f.size();
        if ((n_added + n_updated + n_removed) == 0) {
            ROS_DEBUG_STREAM(_log_prefix << "Received update for entity (" << msg.meta.uid << ") does not apply any changes.");
            return;
        }
        ROS_DEBUG_STREAM(_log_prefix << "Received update for entity (" << msg.meta.uid << ") info: add=" << n_added << ", update=" << n_updated << ", removed=" << n_removed << ".");
        ROS_DEBUG_STREAM_COND(n_added > 0, _log_prefix << " - ADD fields for (" << msg.meta.uid << "): " << entity::build_log_vector(log.new_f) << ".");
        ROS_DEBUG_STREAM_COND(n_updated > 0, _log_prefix << " - UPDATE fields for (" << msg.meta.uid << "): " << entity::build_log_vector(log.updated_f) << ".");
        ROS_DEBUG_STREAM_COND(n_removed > 0, _log_prefix << " - REMOVE fields for (" << msg.meta.uid << "): " << entity::build_log_vector(log.removed_f) << ".");

        // SAVE LOG AND DIFF INTO COLLECTION
        this->ltm_log_insert(log);
        this->ltm_diff_insert(diff);

        // UPDATE CURRENT ENTITY
        this->ltm_update(curr.meta.uid, curr);

        // ADD ENTITIES/LOG TO REGISTRY BY TIMESTAMP
        // TODO: MUTEX HERE?
        RegisterItem2 reg(log.timestamp, log.log_uid, log.entity_uid);
        this->_registry.insert(reg);
    }

    void CrowdEntityPlugin::retrace(EntityMsg &entity, const std::vector<uint32_t> &logs) {
        // ROS_WARN_STREAM("...RETRACING FROM PLUGIN...");
        entity = this->_null_e;

        std::set<std::string> acquired_fields;
        std::set<std::string> remaining_fields = _field_names;
        uint32_t entity_uid = 0;

        std::vector<std::string> all_fields(remaining_fields.begin(), remaining_fields.end());
        // ROS_INFO_STREAM("ALL FIELDS (total=" << all_fields.size() << "): " << ltm::util::vector_to_str(all_fields));

        std::vector<uint32_t>::const_iterator it;
        for (it = logs.begin(); it != logs.end(); ++it) {
            LogType log;
            this->ltm_get_log(*it, log);
            entity_uid = log.entity_uid;
            // ROS_WARN_STREAM("Got log: " << log.log_uid << " for entity " << log.entity_uid);

            EntityWithMetadataPtr diff;
            bool loaded = false;

            std::set<std::string>::iterator f_it;
            for (f_it = remaining_fields.begin(); f_it != remaining_fields.end(); ++f_it) {
                std::string field = *f_it;
                // look for in new, updated and deleted
                bool found = false;

                std::vector<std::string>::iterator s_it;
                s_it = std::find(log.new_f.begin(), log.new_f.end(), field);
                if (s_it != log.new_f.end()) {
                    found = true;
                    // ROS_INFO_STREAM(" - f: " << field << " NEW");
                    // RETRIEVE
                    if (!loaded) {
                        if (!this->ltm_get_diff(log.log_uid, diff)) {
                            ROS_ERROR_STREAM(" - could not load trail entity register #: " << log.log_uid);
                            break;
                        }
                        loaded = true;
                    }
                    this->retrace_retrieve_field(field, diff, entity);
                }
                if (!found) {
                    s_it = std::find(log.updated_f.begin(), log.updated_f.end(), field);
                    if (s_it != log.updated_f.end()) {
                        found = true;
                        // RETRIEVE
                        // ROS_INFO_STREAM(" - f: " << field << " UPDATED");
                        if (!loaded) {
                            if (!this->ltm_get_diff(log.log_uid, diff)) {
                                ROS_ERROR_STREAM(" - could not load trail entity register #: " << log.log_uid);
                                break;
                            }
                            loaded = true;
                        }
                        this->retrace_retrieve_field(field, diff, entity);
                    }
                }
                if (!found) {
                    s_it = std::find(log.removed_f.begin(), log.removed_f.end(), field);
                    if (s_it != log.removed_f.end()) {
                        found = true;
                        // OK (field is already null)
                        // ROS_INFO_STREAM(" - f: " << field << " REMOVED");
                    }
                }
                
                if (found) {
                    acquired_fields.insert(field);
                    remaining_fields.erase(field);

                    if (remaining_fields.empty()) {
                        ROS_INFO_STREAM("Entity (" << entity_uid << ") retrace. Found all fields ("
                                        << acquired_fields.size() << ").");
                        return;
                    }
                } else {
                    // ROS_INFO_STREAM(" - f: " << field << " - not found");
                }
            }    
        }
        // size_t n_acquired = acquired_fields.size();
        // size_t n_missing = remaining_fields.size();
        // std::vector<std::string> acquired(acquired_fields.begin(), acquired_fields.end());
        // std::vector<std::string> missing(remaining_fields.begin(), remaining_fields.end());
        // ROS_INFO_STREAM("Entity (" << entity_uid << ") retrace."
        //                 << "\n - " << n_acquired << " fields were remembered: " << ltm::util::vector_to_str(acquired) 
        //                 << ".\n - " << n_missing << " fields are unknown: " << ltm::util::vector_to_str(missing));
    }

    void CrowdEntityPlugin::retrace_retrieve_field(const std::string& name, EntityWithMetadataPtr &in, EntityMsg &out) {
        EntityMsg _in = *in;

        if (name == "location") { out.location = _in.location; }
        else if (name == "n_people") { out.n_people = _in.n_people; }
        else if (name == "n_male") { out.n_male = _in.n_male; }
        else if (name == "n_female") { out.n_female = _in.n_female; }
        else if (name == "n_children") { out.n_children = _in.n_children; }
        else if (name == "n_adults") { out.n_adults = _in.n_adults; }
        else if (name == "n_elders") { out.n_elders = _in.n_elders; }
    }

    void CrowdEntityPlugin::build_null(EntityMsg &entity) {
        entity.location = "";
        entity.n_people = 0;
        entity.n_male = 0;
        entity.n_female = 0;
        entity.n_children = 0;
        entity.n_adults = 0;
        entity.n_elders = 0;
    }
}