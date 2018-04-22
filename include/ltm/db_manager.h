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

        warehouse_ros_mongo::MongoDatabaseConnection _conn;
        EpisodeCollectionPtr _coll;
        MetadataPtr makeMetadata(EpisodeCollectionPtr coll_ptr, const Episode& episode);

    public:
        Manager(const std::string& name, const std::string& collection, const std::string& host, uint port, float timeout);

        virtual ~Manager();

        void setup();
        bool remove_by_uid(int uid);
        int count();
        bool insert(const Episode& episode);
        bool get(int uid, EpisodeWithMetadataPtr& episode_ptr);
        bool drop_db();
        bool episode_exists(int uid);
        std::string to_short_string(const Episode& episode);
    };

}


#endif //LTM_DB_MANAGER_H
