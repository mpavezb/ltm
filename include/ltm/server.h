#ifndef LTM_SERVER_NODE_H_
#define LTM_SERVER_NODE_H_

// C++
#include <string>
#include <iostream>

// ROS
#include <ros/ros.h>
#include <std_srvs/Empty.h>

// ROS LTM services
#include <ltm/AddEpisode.h>
#include <ltm/GetEpisodes.h>
#include <ltm/QueryServer.h>
#include <ltm/RegisterEpisode.h>
#include <ltm/UpdateTree.h>

// LTM
#include <ltm/db/episode_collection.h>
#include <ltm/plugin/plugins_manager.h>

typedef boost::scoped_ptr<ltm::db::EpisodeCollectionManager> EpisodeCollectionManagerPtr;
typedef boost::scoped_ptr<ltm::plugin::PluginsManager> PluginsManagerPtr;

namespace ltm {

    class Server {

    private:
        // params
        std::string _db_name;
        std::string _db_collection_name;
        std::string _db_host;
        int _db_port;
        float _db_timeout;
        std::string _log_prefix;

        // servers
        ros::ServiceServer _status_service;
        ros::ServiceServer _drop_db_service;
        ros::ServiceServer _add_episode_service;
        ros::ServiceServer _get_episodes_service;
        ros::ServiceServer _query_server_service;
        ros::ServiceServer _register_episode_service;
        ros::ServiceServer _update_tree_service;

        // DB
        EpisodeCollectionManagerPtr _db;

        // Plugins
        PluginsManagerPtr _pl;

        // internal methods
        void show_status();

    public:

        Server();

        virtual ~Server();

        // ==========================================================
        // ROS Services
        // ==========================================================

        /**/
        bool add_episode_service(ltm::AddEpisode::Request  &req, ltm::AddEpisode::Response &res);

        /**/
        bool get_episodes_service(ltm::GetEpisodes::Request  &req, ltm::GetEpisodes::Response &res);

        /**/
        bool query_server_service(ltm::QueryServer::Request  &req, ltm::QueryServer::Response &res);

        /**/
        bool register_episode_service(ltm::RegisterEpisode::Request  &req, ltm::RegisterEpisode::Response &res);

        /**/
        bool update_tree_service(ltm::UpdateTree::Request &req, ltm::UpdateTree::Response &res);

        /**/
        bool status_service(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

        /**/
        bool drop_db_service(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
    };

} /* namespace ltm */

#endif /* LTM_SERVER_NODE_H_ */