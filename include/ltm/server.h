#ifndef LTM_SERVER_NODE_H_
#define LTM_SERVER_NODE_H_


#include <string>
#include <iostream>
#include <ros/ros.h>
#include <ltm/Episode.h>
#include <ltm/AddEpisode.h>
#include <ltm/UpdateEpisode.h>
#include <ltm/GetEpisode.h>
#include <std_srvs/Empty.h>

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

    class Server {

    private:

        // params
        std::string _db_name;
        std::string _db_collection_name;
        std::string _db_host;
        uint _db_port;
        float _db_timeout;

        // servers
        ros::ServiceServer _status_service;
        ros::ServiceServer _drop_db_service;
        ros::ServiceServer _get_episode_service;
        ros::ServiceServer _add_episode_service;
        ros::ServiceServer _update_episode_service;

        // DB
        warehouse_ros_mongo::MongoDatabaseConnection _conn;
        EpisodeCollectionPtr _coll;

        // internal methods
        void setup_db();
        void show_status();
        bool episode_exists(int uid);
        bool remove_by_uid(int uid);
        std::string to_short_string(const ltm::Episode& episode);
        MetadataPtr makeMetadata(EpisodeCollectionPtr coll_ptr, const ltm::Episode& episode);

    public:

        Server();

        virtual ~Server();

        // ==========================================================
        // ROS Services
        // ==========================================================

        /**/
        bool get_episode_service(ltm::GetEpisode::Request  &req, ltm::GetEpisode::Response &res);

        /**/
        bool update_episode_service(ltm::UpdateEpisode::Request  &req, ltm::UpdateEpisode::Response &res);

        /**/
        bool add_episode_service(ltm::AddEpisode::Request  &req, ltm::AddEpisode::Response &res);

        /**/
        bool status_service(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

        /**/
        bool drop_db_service(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
    };

} /* namespace ltm */

#endif /* LTM_SERVER_NODE_H_ */