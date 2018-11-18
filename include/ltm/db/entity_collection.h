#ifndef LTM_PLUGIN_ENTITY_COLLECTION_H
#define LTM_PLUGIN_ENTITY_COLLECTION_H

#include <ros/ros.h>
#include <ltm/db/types.h>
#include <ltm/Episode.h>
#include <ltm/EntityLog.h>
#include <ltm/EntityMetadata.h>
#include <ltm/QueryServer.h>

// get random uid
#include <time.h>
#include <limits>
#include <stdlib.h>

namespace ltm {
    namespace db {

        template<class EntityMsg>
        class EntityCollectionManager {
        private:
            // Entity Types
            typedef ltm_db::MessageWithMetadata<EntityMsg> EntityWithMetadata;
            typedef boost::shared_ptr<const EntityWithMetadata> EntityWithMetadataPtr;

            // Log Message Types
            typedef EntityLog LogType;
            typedef ltm_db::MessageWithMetadata<LogType> LogWithMetadata;
            typedef boost::shared_ptr<const LogWithMetadata> LogWithMetadataPtr;

            // Entity Collection
            typedef ltm_db::MessageCollection<EntityMsg> EntityCollection;
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


            MetadataPtr ltm_make_log_metadata(const ltm::EntityLog &log);
            bool ltm_query_log(const std::string& json, ltm::QueryServer::Response &res);
            bool ltm_query_actual(const std::string& json, ltm::QueryServer::Response &res);

        protected:

            std::string _log_prefix;
            std::string ltm_get_type();
            std::string ltm_get_collection_name();
            std::string ltm_get_log_collection_name();
            std::string ltm_get_status();
            std::string ltm_get_db_name();

            // registry methods
            bool ltm_register_episode(uint32_t uid);
            bool ltm_unregister_episode(uint32_t uid);
            bool ltm_is_reserved(int uid);
            void ltm_get_registry(std::vector<uint32_t> &registry);

            // LOG DB Methods
            int ltm_generate_uid();
            int ltm_reserve_log_uid();
            int ltm_log_count();
            bool ltm_log_has(int uid);
            int ltm_get_last_log_uid(uint32_t entity_uid);
            bool ltm_log_insert(const LogType &log);
            bool ltm_get_log(uint32_t uid, LogType &log);

            // DIFF DB Methods
            int ltm_diff_count();
            bool ltm_diff_has(int uid);
            bool ltm_diff_insert(const EntityMsg &diff);
            bool ltm_get_diff(uint32_t log_uid, EntityWithMetadataPtr &entity_ptr);


            // ENTITY DB Methods
            void ltm_setup_db(DBConnectionPtr db_ptr, std::string db_name, std::string collection_name, std::string type);
            void ltm_resetup_db(const std::string &db_name);
            int ltm_count();
            bool ltm_has(int uid);
            bool ltm_get_last(uint32_t uid, EntityWithMetadataPtr &entity_ptr);
            bool ltm_retrace(uint32_t uid, const ros::Time &stamp, EntityMsg &entity);
            bool ltm_insert(const EntityMsg &entity);
            bool ltm_query(const std::string& json, ltm::QueryServer::Response &res, bool trail);
            bool ltm_update(uint32_t uid, const EntityMsg &entity);
            bool ltm_remove(uint32_t uid);

            bool ltm_drop_db();

            MetadataPtr ltm_create_metadata(const EntityMsg &entity);

            // Must be provided by the user
            virtual MetadataPtr make_metadata(const EntityMsg &entity) = 0;
            virtual void update(const EntityMsg &entity) = 0;
            virtual void retrace(EntityMsg &entity, const std::vector<uint32_t> &logs) = 0;
        };

    }
}

#include <ltm/db/impl/entity_collection_impl.hxx>

#endif //LTM_PLUGIN_ENTITY_COLLECTION_H
