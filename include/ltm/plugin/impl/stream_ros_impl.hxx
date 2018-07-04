#ifndef LTM_PLUGIN_STREAM_ROS_IMPL_HXX
#define LTM_PLUGIN_STREAM_ROS_IMPL_HXX

#include <ltm/plugin/stream_ros.h>
#include <ltm/util/util.h>

namespace ltm {
    namespace plugin {

        template<class StreamType, class StreamSrv>
        void StreamROS<StreamType, StreamSrv>::ltm_setup(const std::string& param_ns, DBConnectionPtr db_ptr, std::string db_name) {
            ltm::util::ParameterServerWrapper psw("~");
            std::string collection_name;
            std::string type;
            psw.getParameter(param_ns + "type", type, "UNKNOWN"); // TODO: use an interesting default value.
            psw.getParameter(param_ns + "collection", collection_name, "UNKNOWN");

            this->_log_prefix = "[LTM][" + type + " Plugin]: ";
            this->ltm_setup_db(db_ptr, db_name, collection_name, type);
        };

        template<class StreamType, class StreamSrv>
        void StreamROS<StreamType, StreamSrv>::ltm_init() {
            ros::NodeHandle priv("~");
            std::string ns = "stream/" + this->ltm_get_type() + "/";
            _status_service = priv.advertiseService(ns + "status", &StreamROS<StreamType, StreamSrv>::status_service, this);
            _drop_db_service = priv.advertiseService(ns + "drop_db", &StreamROS<StreamType, StreamSrv>::drop_db_service, this);
            _add_stream_service = priv.advertiseService(ns + "add", &StreamROS<StreamType, StreamSrv>::add_service, this);
            _get_stream_service = priv.advertiseService(ns + "get", &StreamROS<StreamType, StreamSrv>::get_service, this);
            _delete_stream_service = priv.advertiseService(ns + "delete", &StreamROS<StreamType, StreamSrv>::delete_service, this);
        }

        template<class StreamType, class StreamSrv>
        bool StreamROS<StreamType, StreamSrv>::status_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
            ROS_INFO_STREAM(this->_log_prefix << this->ltm_get_status());
            return true;
        }

        template<class StreamType, class StreamSrv>
        bool StreamROS<StreamType, StreamSrv>::drop_db_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
            ROS_WARN_STREAM(this->_log_prefix << "Deleting all entries from collection '" << this->ltm_get_collection_name() << "'.");
            std_srvs::Empty srv;
            this->ltm_drop_db();
            status_service(srv.request, srv.response);
            return true;
        }

        template<class StreamType, class StreamSrv>
        bool StreamROS<StreamType, StreamSrv>::add_service(StreamSrvRequest &req, StreamSrvResponse &res) {
            typename std::vector<StreamType>::const_iterator it;
            for (it = req.msgs.begin(); it != req.msgs.end(); ++it) {
                this->ltm_insert(*it, this->make_metadata(*it));
            }
            return true;
        }

        template<class StreamType, class StreamSrv>
        bool StreamROS<StreamType, StreamSrv>::get_service(StreamSrvRequest &req, StreamSrvResponse &res) {
            ROS_INFO_STREAM(this->_log_prefix << "Retrieving streams from collection '" << this->ltm_get_collection_name() << "': " << ltm::util::vector_to_str(req.uids));
            res.msgs.clear();

            std::vector<uint32_t> visited, not_found;
            std::vector<uint32_t>::const_iterator it;
            for (it = req.uids.begin(); it != req.uids.end(); ++it) {
                // do not seek repeated msgs
                if (visited.end() != std::find(visited.begin(), visited.end(), *it)) continue;
                visited.push_back(*it);

                StreamWithMetadataPtr stream_ptr;
                if (!this->ltm_get(*it, stream_ptr)) {
                    not_found.push_back(*it);
                    continue;
                }
                res.msgs.push_back(*stream_ptr);
            }
            ROS_WARN_STREAM_COND(not_found.size() > 0, this->_log_prefix
                    << "GET: The following requested streams were not found: " << ltm::util::vector_to_str(not_found));
            return true;
        }

        template<class StreamType, class StreamSrv>
        bool StreamROS<StreamType, StreamSrv>::delete_service(StreamSrvRequest &req, StreamSrvResponse &res) {
            ROS_INFO_STREAM(this->_log_prefix << "Deleting streams from collection '" << this->ltm_get_collection_name() << "': " << ltm::util::vector_to_str(req.uids));
            std::vector<uint32_t>::const_iterator it;
            for (it = req.uids.begin(); it != req.uids.end(); ++it) {
                this->ltm_remove(*it);
            }
            return true;
        }
    }
}

#endif //LTM_PLUGIN_STREAM_ROS_IMPL_HXX
