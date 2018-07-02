#ifndef LTM_DB_EPISODE_H
#define LTM_DB_EPISODE_H

#include <set>
#include <string>
#include <iostream>

#include <ltm/db/types.h>
#include <ltm/Episode.h>
#include <ltm/db/episode_metadata.h>
#include <ltm/db/episode_updater.h>

typedef ltm_db::MessageCollection<ltm::Episode> EpisodeCollection;
typedef boost::shared_ptr<EpisodeCollection> EpisodeCollectionPtr;

typedef ltm_db::MessageWithMetadata<ltm::Episode> EpisodeWithMetadata;
typedef boost::shared_ptr<const EpisodeWithMetadata> EpisodeWithMetadataPtr;

namespace ltm {
    namespace db {

        class EpisodeCollectionManager: EpisodeMetadataBuilder, EpisodeUpdater {
        public:
            DBConnectionPtr _conn;
        private:
            // db params
            std::string _db_name;
            std::string _db_collection_name;
            std::string _db_host;
            uint _db_port;
            float _db_timeout;

            // db handlers
            EpisodeCollectionPtr _coll;

            // reserved uid
            std::set<int> _reserved_uids;
            std::set<int> _db_uids;


            bool update_tree_node(int uid, Episode &updated_episode);

        public:
            EpisodeCollectionManager (const std::string &name, const std::string &collection, const std::string &host, uint port, float timeout);
            virtual ~EpisodeCollectionManager ();

            std::string to_short_string(const Episode &episode);
            void setup();

            // CRUD API
            bool insert(const Episode &episode);
            bool query(const std::string& json, std::vector<uint32_t> &uids);
            bool get(int uid, EpisodeWithMetadataPtr &episode_ptr);
            bool update(const Episode &episode);
            bool remove(int uid);

            // queries
            int reserve_uid();
            int count();
            bool has(int uid);
            bool is_reserved(int uid);
            bool update_tree(int uid);
            bool update_from_children(Episode &episode);
            bool drop_db();
            bool switch_db(const std::string &db_name);
        };

    }
}


#endif //LTM_DB_EPISODE_H
