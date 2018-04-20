#ifndef LTM_SERVER_NODE_H_
#define LTM_SERVER_NODE_H_


#include <string>
#include <iostream>
#include <ros/ros.h>
#include <ltm/Episode.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>


#include <warehouse_ros/message_with_metadata.h>
#include <warehouse_ros_mongo/database_connection.h>

typedef warehouse_ros::MessageWithMetadata<ltm::Episode> EpisodeWithMetadata;
typedef warehouse_ros::MessageCollection<ltm::Episode> EpisodeCollection;

namespace ltm {

    class Server {

    private:
        std::string _name;
        ros::ServiceServer _status_service;
        ros::ServiceServer _append_dummies_service;
        warehouse_ros_mongo::MongoDatabaseConnection _conn;


    public:

        Server();

        virtual ~Server();

//        EpisodeCollection getCollection();

        bool appendDummies(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res);
        bool status(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
    };

} /* namespace ltm */

#endif /* LTM_SERVER_NODE_H_ */