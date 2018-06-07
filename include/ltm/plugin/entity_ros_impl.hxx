#ifndef LTM_PLUGIN_ENTITY_ROS_IMPL_HXX
#define LTM_PLUGIN_ENTITY_ROS_IMPL_HXX

#include <ltm/plugin/entity_ros.h>

namespace ltm {
    namespace plugin {

        template<class EntityType, class EntitySrv>
        void EntityROS<EntityType, EntitySrv>::ltm_setup(const std::string& param_ns, DBConnectionPtr db_ptr, std::string db_name) {
//            ltm::ParameterServerWrapper psw("~");
//            std::string collection_name;
//            std::string type;
//            psw.getParameter(param_ns + "type", type, "UNKNOWN"); // TODO: use an interesting default value.
//            psw.getParameter(param_ns + "collection", collection_name, "UNKNOWN");
//
//            this->_log_prefix = "[LTM][" + type + " Plugin]: ";
//            this->ltm_setup_db(db_ptr, db_name, collection_name, type);
        };

        template<class EntityType, class EntitySrv>
        void EntityROS<EntityType, EntitySrv>::ltm_init() {
//            ros::NodeHandle priv("~");
//            std::string ns = "entity/" + this->ltm_get_type() + "/";
//            _status_service = priv.advertiseService(ns + "status", &EntityROS<EntityType, EntitySrv>::status_service, this);
//            _drop_db_service = priv.advertiseService(ns + "drop_db", &EntityROS<EntityType, EntitySrv>::drop_db_service, this);
//            _add_entity_service = priv.advertiseService(ns + "add", &EntityROS<EntityType, EntitySrv>::add_service, this);
//            _get_entity_service = priv.advertiseService(ns + "get", &EntityROS<EntityType, EntitySrv>::get_service, this);
//            _delete_entity_service = priv.advertiseService(ns + "delete", &EntityROS<EntityType, EntitySrv>::delete_service, this);
        }

        template<class EntityType, class EntitySrv>
        bool EntityROS<EntityType, EntitySrv>::status_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
//            ROS_INFO_STREAM(this->_log_prefix << "Collection '" << this->ltm_get_collection_name() << "' has " << this->ltm_count() << " entries.");
//            return true;
        }

        template<class EntityType, class EntitySrv>
        bool EntityROS<EntityType, EntitySrv>::drop_db_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
//            ROS_WARN_STREAM(this->_log_prefix << "Deleting all entries from collection '" << this->ltm_get_collection_name() << "'.");
//            std_srvs::Empty srv;
//            this->ltm_drop_db();
//            status_service(srv.request, srv.response);
//            return true;
        }

        template<class EntityType, class EntitySrv>
        bool EntityROS<EntityType, EntitySrv>::add_service(EntitySrvRequest &req, EntitySrvResponse &res) {

        }

        template<class EntityType, class EntitySrv>
        bool EntityROS<EntityType, EntitySrv>::get_service(EntitySrvRequest &req, EntitySrvResponse &res) {
//            ROS_INFO_STREAM(this->_log_prefix << "Retrieving entity (" << req.uid << ") from collection '"
//                                              << this->ltm_get_collection_name() << "'");
//            EntityWithMetadataPtr entity_ptr;
//            if (!this->ltm_get(req.uid, entity_ptr)) {
//                ROS_ERROR_STREAM(this->_log_prefix << "Entity with uid '" << req.uid << "' not found.");
//                res.succeeded = (uint8_t) false;
//                return true;
//            }
//            res.msg = *entity_ptr;
//            res.succeeded = (uint8_t) true;
//            return true;
        }

        template<class EntityType, class EntitySrv>
        bool EntityROS<EntityType, EntitySrv>::delete_service(EntitySrvRequest &req, EntitySrvResponse &res) {
//            ROS_INFO_STREAM(this->_log_prefix << "Removing (" << req.uid << ") from collection '" << this->ltm_get_collection_name()
//                                              << "'");
//            res.succeeded = (uint8_t) this->ltm_remove(req.uid);
//            return res.succeeded;
        }
    }
}

#endif //LTM_PLUGIN_ENTITY_ROS_IMPL_HXX
