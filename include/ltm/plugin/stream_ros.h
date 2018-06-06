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
            typedef warehouse_ros::MessageWithMetadata<StreamType> StreamWithMetadata;
            typedef boost::shared_ptr<const StreamWithMetadata> StreamWithMetadataPtr;
            typedef typename StreamSrv::Request StreamSrvRequest;
            typedef typename StreamSrv::Response StreamSrvResponse;

            ros::ServiceServer _status_service;
            ros::ServiceServer _drop_db_service;
            ros::ServiceServer _add_stream_service;
            ros::ServiceServer _get_stream_service;
            ros::ServiceServer _delete_stream_service;
        public:

            void ltm_setup(const std::string& param_ns, DBConnectionPtr db_ptr, std::string db_name) {
                // ROS API
                ltm::ParameterServerWrapper psw("~");
                std::string collection_name;
                std::string type;
                psw.getParameter(param_ns + "type", type, "images");
                psw.getParameter(param_ns + "collection", collection_name, "image_streams");

                this->_log_prefix = "[LTM][" + type + " Plugin]: ";
                this->ltm_setup_db(db_ptr, db_name, collection_name, type);
            }

            void ltm_init() {
                ros::NodeHandle priv("~");
                std::string ns = "stream/" + this->ltm_get_type() + "/";
                _status_service = priv.advertiseService(ns + "status", &StreamROS<StreamType, StreamSrv>::status_service, this);
                _drop_db_service = priv.advertiseService(ns + "drop_db", &StreamROS<StreamType, StreamSrv>::drop_db_service, this);
                _add_stream_service = priv.advertiseService(ns + "add", &StreamROS<StreamType, StreamSrv>::add_service, this);
                _get_stream_service = priv.advertiseService(ns + "get", &StreamROS<StreamType, StreamSrv>::get_service, this);
                _delete_stream_service = priv.advertiseService(ns + "delete", &StreamROS<StreamType, StreamSrv>::delete_service, this);
            }

            // -----------------------------------------------------------------------------------------------------------------
            // ROS API
            // -----------------------------------------------------------------------------------------------------------------

            bool status_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
                ROS_INFO_STREAM(
                        this->_log_prefix << "Collection '" << this->ltm_get_collection_name() << "' has " << this->ltm_count() << " entries.");
                return true;
            }

            bool drop_db_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
                ROS_WARN_STREAM(this->_log_prefix << "Deleting all entries from collection '" << this->ltm_get_collection_name() << "'.");
                std_srvs::Empty srv;
                this->ltm_drop_db();
                status_service(srv.request, srv.response);
                return true;
            }

            bool add_service(StreamSrvRequest &req, StreamSrvResponse &res) {

            }

            bool get_service(StreamSrvRequest &req, StreamSrvResponse &res) {
                ROS_INFO_STREAM(this->_log_prefix << "Retrieving stream (" << req.uid << ") from collection '"
                                            << this->ltm_get_collection_name() << "'");
                StreamWithMetadataPtr stream_ptr;
                if (!this->ltm_get(req.uid, stream_ptr)) {
                    ROS_ERROR_STREAM(this->_log_prefix << "Stream with uid '" << req.uid << "' not found.");
                    res.succeeded = (uint8_t) false;
                    return true;
                }
                res.msg = *stream_ptr;
                res.succeeded = (uint8_t) true;
                return true;
            }

            bool delete_service(StreamSrvRequest &req, StreamSrvResponse &res) {
                ROS_INFO_STREAM(this->_log_prefix << "Removing (" << req.uid << ") from collection '" << this->ltm_get_collection_name()
                                            << "'");
                res.succeeded = (uint8_t) this->ltm_remove(req.uid);
                return res.succeeded;
            }

        };

    }
}

#endif //LTM_PLUGIN_STREAM_ROS_H
