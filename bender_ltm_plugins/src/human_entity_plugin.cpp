#include <bender_ltm_plugins/human_entity_plugin.h>
#include <ltm/util/parameter_server_wrapper.h>
#include <sensor_msgs/Image.h>


namespace bender_ltm_plugins
{
    using namespace ltm::plugin;

    // =================================================================================================================
    // Public API
    // =================================================================================================================

    std::string HumanEntityPlugin::get_type() {
        return this->ltm_get_type();
    }

    void HumanEntityPlugin::initialize(const std::string &param_ns, DBConnectionPtr db_ptr, std::string db_name) {
        _log_prefix = "[LTM][Human Entity]: ";
        ROS_DEBUG_STREAM(_log_prefix << "plugin initialized with ns: " << param_ns);

        build_null(_null_e);

        // all fields
        _field_names.insert("name");
        _field_names.insert("age_bottom");
        _field_names.insert("age_top");
        _field_names.insert("age_avg");
        _field_names.insert("live_phase");
        _field_names.insert("genre");
        // _field_names.insert("face");
        _field_names.insert("emotion");
        _field_names.insert("last_seen");

        // parameters
        ltm::util::ParameterServerWrapper psw("~");
        psw.getParameter(param_ns + "topic", _stm_topic, "/bender/ltm/entity/person/update");

        ros::NodeHandle priv("~");
        _sub = priv.subscribe(_stm_topic, 1, &HumanEntityPlugin::callback, this);

        // DB connection
        this->ltm_setup(param_ns, db_ptr, db_name);

        // init ROS interface
        this->ltm_init();
    }

    HumanEntityPlugin::~HumanEntityPlugin() {

    }

    void HumanEntityPlugin::register_episode(uint32_t uid) {
        this->ltm_register_episode(uid);
        // TODO: WE CAN USE THIS TO DELETE OLD TIMESTAMPS FROM THE REGISTRY
    }

    void HumanEntityPlugin::unregister_episode(uint32_t uid) {
        this->ltm_unregister_episode(uid);
        // TODO: WE CAN USE THIS TO DELETE OLD TIMESTAMPS FROM THE REGISTRY
    }

    void HumanEntityPlugin::collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end) {
        ROS_WARN_STREAM(_log_prefix << "Collecting Human entities for episode " << uid << ".");

        // TODO: mutex (it is not required, because we are using a single-threaded Spinner for callbacks)

        // write entities matching initial and ending times.
        std::map<uint32_t, ltm::EntityRegister> episode_registry;
        std::map<uint32_t, ltm::EntityRegister>::iterator m_it;

        // COMPUTE EPISODE REGISTRY
        int logcnt = 0;
        Registry::const_iterator it;
        for (it = _registry.lower_bound(RegisterItem(_start, 0, 0)); it != _registry.end(); ++it) {
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

    void HumanEntityPlugin::query(const std::string &json, ltm::QueryServer::Response &res, bool trail) {
        this->ltm_query(json, res, trail);
    }

    void HumanEntityPlugin::drop_db() {
        this->reset(this->ltm_get_db_name());
        this->ltm_drop_db();
    }

    void HumanEntityPlugin::reset(const std::string &db_name) {
        this->_registry.clear();
        this->ltm_resetup_db(db_name);
    }

    void HumanEntityPlugin::append_status(std::stringstream &status) {
        status << this->ltm_get_status();
    }

    MetadataPtr HumanEntityPlugin::make_metadata(const EntityMsg &entity) {
        MetadataPtr meta = this->ltm_create_metadata(entity);
        meta->append("name", entity.name);
        meta->append("genre", entity.genre);
        meta->append("age_bottom", entity.age_bottom);
        meta->append("age_top", entity.age_top);
        meta->append("age_avg", entity.age_avg);
        meta->append("live_phase", entity.live_phase);
        meta->append("emotion", entity.emotion);

        double last_seen = entity.last_seen.sec + entity.last_seen.nsec * pow10(-9);
        meta->append("last_seen", last_seen);

        return meta;
    }

    // =================================================================================================================
    // Private API
    // =================================================================================================================

    void HumanEntityPlugin::callback(const EntityMsg &msg) {
        this->update(msg);
    }

    void HumanEntityPlugin::update(const EntityMsg& msg) {
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
            curr.name = curr_with_md->name;
            curr.age_bottom = curr_with_md->age_bottom;
            curr.age_top = curr_with_md->age_top;
            curr.age_avg = curr_with_md->age_avg;
            curr.live_phase = curr_with_md->live_phase;
            curr.genre = curr_with_md->genre;
            curr.face = curr_with_md->face;
            curr.emotion = curr_with_md->emotion;
            curr.last_seen = curr_with_md->last_seen;
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
        entity::update_field<std::string>(log, "name", curr.name, diff.name, msg.name, _null_e.name);
        entity::update_field<uint8_t>(log, "age_bottom", curr.age_bottom, diff.age_bottom, msg.age_bottom, _null_e.age_bottom);
        entity::update_field<uint8_t>(log, "age_top", curr.age_top, diff.age_top, msg.age_top, _null_e.age_top);
        entity::update_field<uint8_t>(log, "age_avg", curr.age_avg, diff.age_avg, msg.age_avg, _null_e.age_avg);
        entity::update_field<uint8_t>(log, "live_phase", curr.live_phase, diff.live_phase, msg.live_phase, _null_e.live_phase);
        entity::update_field<uint8_t>(log, "genre", curr.genre, diff.genre, msg.genre, _null_e.genre);
        entity::update_field<sensor_msgs::Image>(log, "face", curr.face, diff.face, msg.face, _null_e.face);
        entity::update_field<std::string>(log, "emotion", curr.emotion, diff.emotion, msg.emotion, _null_e.emotion);
        entity::update_field<ros::Time>(log, "last_seen", curr.last_seen, diff.last_seen, msg.last_seen, _null_e.last_seen);

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
        RegisterItem reg(log.timestamp, log.log_uid, log.entity_uid);
        this->_registry.insert(reg);
    }

    void HumanEntityPlugin::retrace(EntityMsg &entity, const std::vector<uint32_t> &logs) {
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

    void HumanEntityPlugin::retrace_retrieve_field(const std::string& name, EntityWithMetadataPtr &in, EntityMsg &out) {
        EntityMsg _in = *in;

        if (name == "name") { out.name = _in.name; }
        else if (name == "genre") { out.genre = _in.genre; }
        else if (name == "age_bottom") { out.age_bottom = _in.age_bottom; }
        else if (name == "age_top") { out.age_top = _in.age_top; }
        else if (name == "age_avg") { out.age_avg = _in.age_avg; }
        else if (name == "live_phase") { out.live_phase = _in.live_phase; }
        else if (name == "emotion") { out.emotion = _in.emotion; }
        else if (name == "face") { out.face = _in.face; }
        else if (name == "last_seen") { out.last_seen = _in.last_seen; }
    }

    void HumanEntityPlugin::build_null(EntityMsg &entity) {
        entity.name = "";
        entity.genre = 0;
        entity.age_bottom = 0;
        entity.age_top = 0;
        entity.age_avg = 0;
        entity.live_phase = 0;
        entity.face = sensor_msgs::Image();
        entity.emotion = "";
        entity.last_seen = ros::Time(0);
    }
}