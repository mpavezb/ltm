#ifndef LTM_DB_EPISODE_H
#define LTM_DB_EPISODE_H

#include <set>
#include <string>
#include <iostream>

#include <ltm/db/types.h>
#include <ltm/Episode.h>

typedef ltm_db::MessageCollection<ltm::Episode> EpisodeCollection;
typedef boost::shared_ptr<EpisodeCollection> EpisodeCollectionPtr;

typedef ltm_db::MessageWithMetadata<ltm::Episode> EpisodeWithMetadata;
typedef boost::shared_ptr<const EpisodeWithMetadata> EpisodeWithMetadataPtr;

namespace ltm {
    namespace db {

        class EpisodeCollectionManager {
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

            // metadata methods
            // -------------------------------------------------------------------------------------------------------------
            MetadataPtr make_metadata(const Episode &episode);
            void make_meta_episode(const Episode &node, MetadataPtr meta);
            void make_meta_info(const Info &node, MetadataPtr meta);
            void make_meta_when(const When &node, MetadataPtr meta);
            void make_meta_where(const Where &node, MetadataPtr meta);
            void make_meta_what(const What &node, MetadataPtr meta);
            void make_meta_relevance(const Relevance &node, MetadataPtr meta);
            void make_meta_relevance_historical(const HistoricalRelevance &node, MetadataPtr meta);
            void make_meta_relevance_emotional(const EmotionalRelevance &node, MetadataPtr meta);

            // update tree methods
            // -------------------------------------------------------------------------------------------------------------
            bool update_tree_node(int uid, Episode &updated_episode);

            // init methods
            void update_tree_init(Episode &node);
            void update_tree_tags_init(Episode &node);
            void update_tree_info_init(Info &node);
            void update_tree_when_init(When &node);
            void update_tree_where_init(Where &node);
            void update_tree_what_init(What &node);
            void update_tree_relevance_init(Relevance &node);
            void update_tree_relevance_historical_init(HistoricalRelevance &node);
            void update_tree_relevance_emotional_init(EmotionalRelevance &node);

            // add child information methods
            bool update_tree_tags(Episode &node, const Episode &child);
            bool update_tree_info(Info &node, const Info &child);
            bool update_tree_when(When &node, const When &child);
            bool update_tree_where(Where &node, const Where &child, bool is_leaf, int node_uid, int child_uid, std::vector<geometry_msgs::Point> &positions);
            bool update_tree_what(What &node, const What &child);
            bool update_tree_what_streams(What &node, const What &child);
            bool update_tree_what_entities(What &node, const What &child);
            bool update_tree_relevance(Relevance &node, const Relevance &child, bool is_leaf);
            bool update_tree_relevance_historical(HistoricalRelevance &node, const HistoricalRelevance &child);
            bool update_tree_relevance_emotional(EmotionalRelevance &node, const EmotionalRelevance &child, bool is_leaf);

            // ending methods
            bool update_tree_last(Episode &node, std::vector<geometry_msgs::Point> &positions);
            bool update_tree_where_last(Where &node, std::vector<geometry_msgs::Point> &positions);

            // tools
            // -------------------------------------------------------------------------------------------------------------
            void uid_vector_merge(std::vector<uint32_t> &result, const std::vector<uint32_t> &source);
            void vector_merge(std::vector<std::string> &result, const std::vector<std::string> &source);
            std::string vector_to_str(const std::vector<std::string> &array);


        public:
            EpisodeCollectionManager (const std::string &name, const std::string &collection, const std::string &host, uint port, float timeout);
            virtual ~EpisodeCollectionManager ();

            std::string to_short_string(const Episode &episode);
            void setup();

            // CRUD API
            bool insert(const Episode &episode);
            bool get(int uid, EpisodeWithMetadataPtr &episode_ptr);
            bool update(const Episode &episode);
            bool remove(int uid);

            // queries
            int reserve_uid();
            int count();
            bool has(int uid);
            bool is_reserved(int uid);
            bool update_tree(int uid);
            bool drop_db();
            bool update_from_children(Episode &episode);
            bool update_from_child(Episode &node, const Episode &child, std::vector<geometry_msgs::Point> &positions);

        };

    }
}


#endif //LTM_DB_EPISODE_H
