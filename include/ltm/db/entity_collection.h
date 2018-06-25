#ifndef LTM_PLUGIN_ENTITY_COLLECTION_H
#define LTM_PLUGIN_ENTITY_COLLECTION_H

#include <ros/ros.h>
#include <ltm/db/types.h>
#include <ltm/Episode.h>
#include <ltm/EntityLog.h>

// get random uid
#include <time.h>
#include <limits>
#include <stdlib.h>

namespace ltm {
    namespace db {

        template<class EntityType>
        class EntityCollectionManager {
        private:
            // Entity Types
            typedef ltm_db::MessageWithMetadata<EntityType> EntityWithMetadata;
            typedef boost::shared_ptr<const EntityWithMetadata> EntityWithMetadataPtr;

            // Log Message Types
            typedef EntityLog LogType;
            typedef ltm_db::MessageWithMetadata<LogType> LogWithMetadata;
            typedef boost::shared_ptr<const LogWithMetadata> LogWithMetadataPtr;

            // Entity Collection
            typedef ltm_db::MessageCollection<EntityType> EntityCollection;
            typedef boost::shared_ptr<EntityCollection> EntityCollectionPtr;

            // Log Message Collection
            typedef ltm_db::MessageCollection<LogType> LogCollection;
            typedef boost::shared_ptr<LogCollection> LogCollectionPtr;

            // database connection
            EntityCollectionPtr _coll;
            EntityCollectionPtr _diff_coll;
            LogCollectionPtr _log_coll;
            DBConnectionPtr _conn;

            // database parameters
            std::string _collection_name;
            std::string _diff_collection_name;
            std::string _log_collection_name;
            std::string _db_name;
            std::string _type;

            // control
            std::vector<uint32_t> _registry;
            std::set<int> _reserved_log_uids;
            std::set<int> _log_uids_cache;

        protected:

            std::string _log_prefix;
            std::string ltm_get_type();
            std::string ltm_get_collection_name();
            std::string ltm_get_status();

            // registry methods
            bool ltm_register_episode(uint32_t uid);
            bool ltm_unregister_episode(uint32_t uid);
            bool ltm_is_reserved(int uid);
            void ltm_get_registry(std::vector<uint32_t> registry);

            // LOG DB Methods
            int ltm_reserve_log_uid();
            int ltm_log_count();
            bool ltm_log_has(int uid);
            int ltm_get_last_log_uid(uint32_t entity_uid);
            bool ltm_log_insert(const LogType &log, MetadataPtr metadata);

            // DIFF DB Methods
            int ltm_diff_count();
            bool ltm_diff_has(int uid);
            bool ltm_diff_insert(const EntityType &diff, MetadataPtr metadata);

            // ENTITY DB Methods
            void ltm_setup_db(DBConnectionPtr db_ptr, std::string db_name, std::string collection_name, std::string type);
            void ltm_resetup_db();
            int ltm_count();
            bool ltm_has(int uid);
            bool ltm_get(uint32_t uid, EntityWithMetadataPtr &entity_ptr);

            bool ltm_remove(uint32_t uid);
            bool ltm_drop_db();

            bool ltm_insert(const EntityType &entity, MetadataPtr metadata);
            MetadataPtr ltm_create_metadata();
            bool ltm_update(uint32_t uid, const EntityType &entity, MetadataPtr metadata);
        };

    }
}

#include <ltm/db/impl/entity_collection_impl.hxx>

#endif //LTM_PLUGIN_ENTITY_COLLECTION_H
