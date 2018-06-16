#ifndef LTM_PLUGIN_ENTITY_ROS_H
#define LTM_PLUGIN_ENTITY_ROS_H

#include <ros/ros.h>
#include <ltm/Episode.h>
#include <std_srvs/Empty.h>
#include <ltm/db/entity_collection.h>
#include <ltm/parameter_server_wrapper.h>

namespace ltm {
    namespace plugin {

        template<class EntityType, class EntitySrv>
        class EntityROS : public ltm::db::EntityCollectionManager<EntityType> {
        private:
            // Entity Types
            typedef ltm_db::MessageWithMetadata<EntityType> EntityWithMetadata;
            typedef boost::shared_ptr<const EntityWithMetadata> EntityWithMetadataPtr;

            // Log Message Types
            typedef EntityLog LogType;
            typedef ltm_db::MessageWithMetadata<LogType> LogWithMetadata;
            typedef boost::shared_ptr<const LogWithMetadata> LogWithMetadataPtr;

            typedef typename EntitySrv::Request EntitySrvRequest;
            typedef typename EntitySrv::Response EntitySrvResponse;

            ros::ServiceServer _status_service;
            ros::ServiceServer _drop_db_service;
            ros::ServiceServer _add_entity_service;
            ros::ServiceServer _get_entity_service;
            ros::ServiceServer _delete_entity_service;

        public:
            void ltm_setup(const std::string &param_ns, DBConnectionPtr db_ptr, std::string db_name);

            void ltm_init();

        private:
            bool status_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

            bool drop_db_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

            bool add_service(EntitySrvRequest &req, EntitySrvResponse &res);

            bool get_service(EntitySrvRequest &req, EntitySrvResponse &res);

            bool delete_service(EntitySrvRequest &req, EntitySrvResponse &res);

        };
    }
}

#include <ltm/plugin/entity_ros_impl.hxx>

#endif //LTM_PLUGIN_ENTITY_ROS_H
