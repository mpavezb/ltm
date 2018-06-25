#ifndef LTM_PLUGIN_STREAM_COLLECTION_H
#define LTM_PLUGIN_STREAM_COLLECTION_H

#include <ros/ros.h>
#include <ltm/db/types.h>
#include <ltm/Episode.h>

namespace ltm {
    namespace db {

        template<class StreamType>
        class StreamCollectionManager {
        private:
            typedef ltm_db::MessageCollection<StreamType> StreamCollection;
            typedef boost::shared_ptr<StreamCollection> StreamCollectionPtr;

            typedef ltm_db::MessageWithMetadata<StreamType> StreamWithMetadata;
            typedef boost::shared_ptr<const StreamWithMetadata> StreamWithMetadataPtr;

            // database connection
            StreamCollectionPtr _coll;
            DBConnectionPtr _conn;

            // database parameters
            std::string _collection_name;
            std::string _db_name;
            std::string _type;

            // control
            std::vector<uint32_t> _registry;

        public:
            std::string _log_prefix;

            std::string ltm_get_type();
            std::string ltm_get_collection_name();
            std::string ltm_get_status();

            void ltm_setup_db(DBConnectionPtr db_ptr, std::string db_name, std::string collection_name, std::string type);
            void ltm_resetup_db();
            bool ltm_register_episode(uint32_t uid);
            bool ltm_unregister_episode(uint32_t uid);
            bool ltm_is_reserved(int uid);
            bool ltm_remove(uint32_t uid);
            bool ltm_has(int uid);
            int ltm_count();
            bool ltm_drop_db();
            bool ltm_get(uint32_t uid, StreamWithMetadataPtr &stream_ptr);
            bool ltm_insert(const StreamType &stream, MetadataPtr metadata);
            MetadataPtr ltm_create_metadata();
            bool ltm_update(uint32_t uid, const StreamType &stream);

        };

    }
}

#include <ltm/db/impl/stream_collection_impl.hxx>

#endif //LTM_PLUGIN_STREAM_COLLECTION_H
