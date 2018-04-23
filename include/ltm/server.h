#ifndef LTM_SERVER_NODE_H_
#define LTM_SERVER_NODE_H_


#include <string>
#include <iostream>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <ltm/AddEpisode.h>
#include <ltm/UpdateTree.h>
#include <ltm/GetEpisode.h>
#include <ltm/db_manager.h>

typedef boost::scoped_ptr<ltm::Manager> ManagerPtr;

namespace ltm {

    class Server {

    private:

        // params
        std::string _db_name;
        std::string _db_collection_name;
        std::string _db_host;
        int _db_port;
        float _db_timeout;

        // servers
        ros::ServiceServer _status_service;
        ros::ServiceServer _drop_db_service;
        ros::ServiceServer _get_episode_service;
        ros::ServiceServer _add_episode_service;
        ros::ServiceServer _update_tree_service;

        // DB
        ManagerPtr _db;

        // internal methods
        void show_status();

    public:

        Server();

        virtual ~Server();

        // ==========================================================
        // ROS Services
        // ==========================================================

        /**/
        bool get_episode_service(ltm::GetEpisode::Request  &req, ltm::GetEpisode::Response &res);

        /**/
        bool update_tree_service(ltm::UpdateTree::Request &req, ltm::UpdateTree::Response &res);

        /**/
        bool add_episode_service(ltm::AddEpisode::Request  &req, ltm::AddEpisode::Response &res);

        /**/
        bool status_service(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

        /**/
        bool drop_db_service(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
    };

} /* namespace ltm */

#endif /* LTM_SERVER_NODE_H_ */