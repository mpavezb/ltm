#ifndef LTM_PLUGIN_ENTITY_ROS_IMPL_HXX
#define LTM_PLUGIN_ENTITY_ROS_IMPL_HXX

#include <ltm/plugin/entity_ros.h>
#include <ltm/util/util.h>

namespace ltm {
    namespace plugin {

        template<class EntityMsg, class EntitySrv>
        void EntityROS<EntityMsg, EntitySrv>::ltm_setup(const std::string& param_ns, DBConnectionPtr db_ptr, std::string db_name) {
            ltm::util::ParameterServerWrapper psw("~");
            std::string collection_name;
            std::string type;
            psw.getParameter(param_ns + "type", type, "UNKNOWN");
            psw.getParameter(param_ns + "collection", collection_name, "UNKNOWN");

            this->_log_prefix = "[LTM][" + type + " Plugin]: ";
            this->ltm_setup_db(db_ptr, db_name, collection_name, type);
        };

        template<class EntityMsg, class EntitySrv>
        void EntityROS<EntityMsg, EntitySrv>::ltm_init() {
            ros::NodeHandle priv("~");
            std::string ns = "entity/" + this->ltm_get_type() + "/";
            _status_service = priv.advertiseService(ns + "status", &EntityROS<EntityMsg, EntitySrv>::status_service, this);
            _drop_db_service = priv.advertiseService(ns + "drop_db", &EntityROS<EntityMsg, EntitySrv>::drop_db_service, this);
            _add_entity_service = priv.advertiseService(ns + "add", &EntityROS<EntityMsg, EntitySrv>::add_service, this);
            _get_entity_service = priv.advertiseService(ns + "get", &EntityROS<EntityMsg, EntitySrv>::get_service, this);
            _delete_entity_service = priv.advertiseService(ns + "delete", &EntityROS<EntityMsg, EntitySrv>::delete_service, this);
            _get_entity_logs_service = priv.advertiseService(ns + "get_logs", &EntityROS<EntityMsg, EntitySrv>::get_logs_service, this);
        }

        template<class EntityMsg, class EntitySrv>
        bool EntityROS<EntityMsg, EntitySrv>::status_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
            ROS_INFO_STREAM(this->_log_prefix << this->ltm_get_status());
            return true;
        }

        template<class EntityMsg, class EntitySrv>
        bool EntityROS<EntityMsg, EntitySrv>::drop_db_service(ltm::DropDB::Request &req, ltm::DropDB::Response &res) {
            if (!req.i_understand_this_is_a_dangerous_operation ||
                req.html_is_a_real_programming_language) {
                ROS_WARN_STREAM(this->_log_prefix << "Attempted to drop the collections for entity '" << this->ltm_get_type()
                                            << "' with unmet requirements!. Please read the ltm/DropDB.srv description. Bye!.");
                return false;
            }

            ROS_WARN_STREAM(this->_log_prefix << "Deleting all entries from collection '" << this->ltm_get_collection_name() << "'.");
            std_srvs::Empty srv;
            this->ltm_drop_db();
            status_service(srv.request, srv.response);
            return true;
        }

        template<class EntityMsg, class EntitySrv>
        bool EntityROS<EntityMsg, EntitySrv>::add_service(EntitySrvRequest &req, EntitySrvResponse &res) {
            typename std::vector<EntityMsg>::const_iterator it;
            for (it = req.msgs.begin(); it != req.msgs.end(); ++it) {
                this->update(*it);
            }
            return true;
        }

        template<class EntityMsg, class EntitySrv>
        bool EntityROS<EntityMsg, EntitySrv>::get_service(EntitySrvRequest &req, EntitySrvResponse &res) {
            ROS_INFO_STREAM(this->_log_prefix << "Retrieving entities from collection '" << this->ltm_get_collection_name() << "': " << ltm::util::vector_to_str(req.uids));
            res.msgs.clear();

            // check
            if (!req.stamps.empty() && req.stamps.size() != req.uids.size()) {
                ROS_WARN_STREAM("Get Service: stamps must be empty or be the same size as the uids field.");
                return false;
            }

            // build dummy vector if required
            if (req.stamps.empty()) {
                ros::Time now = ros::Time::now();
                for (int i=0; i<req.uids.size(); ++i) {
                    req.stamps.push_back(now);
                }
            }

            // search
            std::vector<uint32_t> visited, not_found;
            std::vector<uint32_t>::const_iterator it;
            std::vector<ros::Time>::const_iterator t_it;
            for (it = req.uids.begin(), t_it = req.stamps.begin(); it != req.uids.end(); ++it, ++t_it) {
                // do not seek repeated msgs
                if (visited.end() != std::find(visited.begin(), visited.end(), *it)) continue;
                visited.push_back(*it);

                // lookup
                // ROS_DEBUG_STREAM("RETRACE: Looking for uid (" << *it << ") at time: " << (*t_it).sec);
                EntityMsg entity;
                if (!this->ltm_retrace(*it, *t_it, entity)) {
                    not_found.push_back(*it);
                    // ROS_DEBUG_STREAM("RETRACE: NOT FOUND");
                    continue;
                }
                // ROS_WARN_STREAM("RETRACE: FOUND!");
                res.msgs.push_back(entity);
            }
            ROS_WARN_STREAM_COND(not_found.size() > 0, this->_log_prefix
                    << "GET: The following requested entities were not found: " << ltm::util::vector_to_str(not_found));
            return true;
        }

        template<class EntityMsg, class EntitySrv>
        bool EntityROS<EntityMsg, EntitySrv>::delete_service(EntitySrvRequest &req, EntitySrvResponse &res) {
            ROS_INFO_STREAM(this->_log_prefix << "Deleting entities from collection '" << this->ltm_get_collection_name() << "': " << ltm::util::vector_to_str(req.uids));

            ltm::QueryServer::Response qr;
//            this->ltm_query()
            // TODO: delete LOGs
            std::vector<uint32_t>::const_iterator it;
            for (it = req.uids.begin(); it != req.uids.end(); ++it) {
                this->ltm_remove(*it);
            }
            return true;
        }

        template<class EntityMsg, class EntitySrv>
        bool EntityROS<EntityMsg, EntitySrv>::get_logs_service(ltm::GetEntityLogs::Request &req, ltm::GetEntityLogs::Response &res) {
            ROS_INFO_STREAM(this->_log_prefix << "Retrieving entity logs from collection '" << this->ltm_get_log_collection_name() << "': " << ltm::util::vector_to_str(req.uids));
            res.logs.clear();

            // TODO: do domething about repeated logs

            // search
            std::vector<uint32_t>::const_iterator it;
            for (it = req.uids.begin(); it != req.uids.end(); ++it) {

                uint32_t log_uid = *it;
                
                // lookup
                ltm::EntityLog log;
                if (!this->ltm_get_log(log_uid, log)) {
                    ROS_WARN_STREAM(this->_log_prefix << "get_logs service: EntityLog with uid (" << log_uid << ") not found on collection '" << this->ltm_get_log_collection_name() << "'.");
                    continue;
                }
                res.logs.push_back(log);
            }
            return true;            
        }
    }
}

#endif //LTM_PLUGIN_ENTITY_ROS_IMPL_HXX
