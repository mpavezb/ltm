#ifndef LTM_PLUGIN_ENTITY_COLLECTION_H
#define LTM_PLUGIN_ENTITY_COLLECTION_H

#include <ros/ros.h>
#include <ltm/db/types.h>
#include <ltm/Episode.h>

namespace ltm {
    namespace db {

        template<class EntityType>
        class EntityCollectionManager {
        private:
            typedef warehouse_ros::MessageCollection<EntityType> EntityCollection;
            typedef boost::shared_ptr<EntityCollection> EntityCollectionPtr;

            typedef warehouse_ros::MessageWithMetadata<EntityType> EntityWithMetadata;
            typedef boost::shared_ptr<const EntityWithMetadata> EntityWithMetadataPtr;

            // database connection
            EntityCollectionPtr _coll;
            EntityCollectionPtr _log_coll;
            DBConnectionPtr _conn;

            // database parameters
            std::string _collection_name;
            std::string _log_collection_name;
            std::string _db_name;
            std::string _type;

            // control
            std::vector<uint32_t> _registry;

        public:
            std::string _log_prefix;

            std::string ltm_get_type();
            std::string ltm_get_collection_name();
            void ltm_setup_db(DBConnectionPtr db_ptr, std::string db_name, std::string collection_name, std::string type);
            bool ltm_register_episode(uint32_t uid);
            bool ltm_unregister_episode(uint32_t uid);
            bool ltm_is_reserved(int uid);
            bool ltm_remove(uint32_t uid);
            bool ltm_has(int uid);
            int ltm_count();
            bool ltm_drop_db();
            bool ltm_get(uint32_t uid, EntityWithMetadataPtr &entity_ptr);
            bool ltm_insert(const EntityType &entity, MetadataPtr metadata);
            MetadataPtr ltm_create_metadata();
            bool ltm_update(uint32_t uid, const EntityType &entity);

        };

    }
}

#include <ltm/db/entity_collection_impl.hxx>

#endif //LTM_PLUGIN_ENTITY_COLLECTION_H
