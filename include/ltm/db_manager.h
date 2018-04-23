#ifndef LTM_DB_MANAGER_H
#define LTM_DB_MANAGER_H

#include <string>
#include <iostream>
#include <ltm/Episode.h>
#include <warehouse_ros/message_with_metadata.h>
#include <warehouse_ros_mongo/database_connection.h>


typedef warehouse_ros::MessageCollection<ltm::Episode> EpisodeCollection;
typedef boost::shared_ptr<EpisodeCollection> EpisodeCollectionPtr;

typedef warehouse_ros::MessageWithMetadata<ltm::Episode> EpisodeWithMetadata;
typedef boost::shared_ptr<const EpisodeWithMetadata> EpisodeWithMetadataPtr;

typedef warehouse_ros_mongo::Query Query;
typedef warehouse_ros_mongo::Query::Ptr QueryPtr;

typedef warehouse_ros::Metadata Metadata;
typedef warehouse_ros::Metadata::Ptr MetadataPtr;

namespace ltm {

    class Manager {
    private:
        // db params
        std::string _db_name;
        std::string _db_collection_name;
        std::string _db_host;
        uint _db_port;
        float _db_timeout;

        // db handlers
        warehouse_ros_mongo::MongoDatabaseConnection _conn;
        EpisodeCollectionPtr _coll;

        // metadata methods
        MetadataPtr make_metadata(const Episode& episode);
        void make_meta_episode(const Episode& node, MetadataPtr meta);
        void make_meta_info(const Info& node, MetadataPtr meta);
        void make_meta_when(const When& node, MetadataPtr meta);
        void make_meta_where(const Where& node, MetadataPtr meta);
        void make_meta_what(const What& node, MetadataPtr meta);
        void make_meta_relevance(const Relevance& node, MetadataPtr meta);
        void make_meta_relevance_historical(const HistoricalRelevance& node, MetadataPtr meta);
        void make_meta_relevance_emotional(const EmotionalRelevance& node, MetadataPtr meta);

        // update tree methods
        bool update_tree_node(int uid, Episode& updated_episode);
        bool update_tree_tags(Episode& node, const Episode& child, bool first, bool is_leaf);
        bool update_tree_info(Info& node, const Info& child, bool first, bool is_leaf);
        bool update_tree_when(When& node, const When& child, bool first);
        bool update_tree_where(Where& node, const Where& child, bool first, bool is_leaf);
        bool update_tree_what(What& node, const What& child, bool first, bool is_leaf);
        bool update_tree_relevance(Relevance& node, const Relevance& child, bool first, bool is_leaf);
        bool update_tree_relevance_historical(HistoricalRelevance& node, const HistoricalRelevance& child, bool first);
        bool update_tree_relevance_emotional(EmotionalRelevance& node, const EmotionalRelevance& child, bool first, bool is_leaf);


    public:
        Manager(const std::string& name, const std::string& collection, const std::string& host, uint port, float timeout);
        virtual ~Manager();

        std::string to_short_string(const Episode& episode);

        void setup();

        // CRUD API
        bool insert(const Episode& episode);
        bool get(int uid, EpisodeWithMetadataPtr& episode_ptr);
        bool update(const Episode& episode);
        bool remove(int uid);

        // queries
        int count();
        bool has(int uid);
        bool update_tree(int uid);
        bool drop_db();
    };

}


#endif //LTM_DB_MANAGER_H
