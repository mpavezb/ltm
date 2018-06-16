#ifndef LTM_PLUGIN_STREAM_ROS_H
#define LTM_PLUGIN_STREAM_ROS_H

#include <ros/ros.h>
#include <ltm/Episode.h>
#include <std_srvs/Empty.h>
#include <ltm/db/stream_collection.h>
#include <ltm/parameter_server_wrapper.h>

namespace ltm {
    namespace plugin {

        template<class StreamType, class StreamSrv>
        class StreamROS : public ltm::db::StreamCollectionManager<StreamType> {
        private:
            typedef ltm_db::MessageWithMetadata<StreamType> StreamWithMetadata;
            typedef boost::shared_ptr<const StreamWithMetadata> StreamWithMetadataPtr;
            typedef typename StreamSrv::Request StreamSrvRequest;
            typedef typename StreamSrv::Response StreamSrvResponse;

            ros::ServiceServer _status_service;
            ros::ServiceServer _drop_db_service;
            ros::ServiceServer _add_stream_service;
            ros::ServiceServer _get_stream_service;
            ros::ServiceServer _delete_stream_service;

        public:
            void ltm_setup(const std::string &param_ns, DBConnectionPtr db_ptr, std::string db_name);

            void ltm_init();

        private:
            bool status_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

            bool drop_db_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

            bool add_service(StreamSrvRequest &req, StreamSrvResponse &res);

            bool get_service(StreamSrvRequest &req, StreamSrvResponse &res);

            bool delete_service(StreamSrvRequest &req, StreamSrvResponse &res);

        };
    }
}

#include <ltm/plugin/stream_ros_impl.hxx>

#endif //LTM_PLUGIN_STREAM_ROS_H
