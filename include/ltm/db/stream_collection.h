#ifndef LTM_STREAM_COLLECTION_H
#define LTM_STREAM_COLLECTION_H

#include <set>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ltm/db/types.h>
#include <ltm/Episode.h>
#include <std_srvs/Empty.h>

namespace ltm {
    namespace db {

        template<class StreamType, class StreamSrv>
        class StreamCollectionManager {
        private:
            ros::ServiceServer _status_service;
            ros::ServiceServer _drop_db_service;
            ros::ServiceServer _add_stream_service;
            ros::ServiceServer _get_stream_service;
            ros::ServiceServer _delete_stream_service;

        public:
            typedef typename StreamSrv::Request StreamSrvRequest;
            typedef typename StreamSrv::Response StreamSrvResponse;

            typedef warehouse_ros::MessageCollection<StreamType> StreamCollection;
            typedef boost::shared_ptr<StreamCollection> StreamCollectionPtr;

            typedef warehouse_ros::MessageWithMetadata<StreamType> StreamWithMetadata;
            typedef boost::shared_ptr<const StreamWithMetadata> StreamWithMetadataPtr;


            StreamCollectionManager(DBConnectionPtr ptr, std::string db_name, std::string collection_name,
                                    std::string type, std::string log_prefix) {
                _conn = ptr;
                _db_name = db_name;
                _collection_name = collection_name;
                _type = type;
                _log_prefix = log_prefix;
            }


            // database connection
            StreamCollectionPtr _coll;
            DBConnectionPtr _conn;

            // database parameters
            std::string _collection_name;
            std::string _db_name;

            // stream data
            std::string _type;

            // control
            std::vector<uint32_t> _registry;
            std::string _log_prefix;


            void subscribe() {}

            void unsubscribe() {}

            void setup() {
                setup_db();

                // ROS API
                ros::NodeHandle priv("~");
                _status_service = priv.advertiseService("stream/" + _type + "/status", &StreamCollectionManager<StreamType, StreamSrv>::status_service, this);
                _drop_db_service = priv.advertiseService("stream/" + _type + "/drop_db", &StreamCollectionManager<StreamType, StreamSrv>::drop_db_service, this);
                _add_stream_service = priv.advertiseService("stream/" + _type + "/add", &StreamCollectionManager<StreamType, StreamSrv>::add_service, this);
                _get_stream_service = priv.advertiseService("stream/" + _type + "/get", &StreamCollectionManager<StreamType, StreamSrv>::get_service, this);
                _delete_stream_service = priv.advertiseService("stream/" + _type + "/delete", &StreamCollectionManager<StreamType, StreamSrv>::delete_service, this);
            }

            void register_episode(uint32_t uid) {
                ROS_DEBUG_STREAM(_log_prefix << "LTM Image Stream plugin: registering episode " << uid);

                // subscribe on demand
                if (_registry.empty()) subscribe();

                // register in cache
                std::vector<uint32_t>::const_iterator it = std::find(_registry.begin(), _registry.end(), uid);
                if (it == _registry.end()) _registry.push_back(uid);
            }

            void unregister_episode(uint32_t uid) {
                ROS_DEBUG_STREAM(_log_prefix << "LTM Image Stream plugin: unregistering episode " << uid);

                // unregister from cache
                std::vector<uint32_t>::iterator it = std::find(_registry.begin(), _registry.end(), uid);
                if (it != _registry.end()) _registry.erase(it);

                // unsubscribe on demand
                if (_registry.empty()) unsubscribe();
            }

            std::string get_type() {
                return _type;
            }

            std::string get_collection_name() {
                return _collection_name;
            }

            bool is_reserved(int uid) {
                // value is in registry
                std::vector<uint32_t>::iterator it = std::find(_registry.begin(), _registry.end(), uid);
                if (it != _registry.end()) return true;

                // TODO: value is in db cache

                // value is already in DB
                return has(uid);
            }

            // DB handling methods
            void setup_db() {
                try {
                    // host, port, timeout
                    _coll = _conn->openCollectionPtr<StreamType>(_db_name, _collection_name);
                }
                catch (const warehouse_ros::DbConnectException &exception) {
                    // Connection timeout
                    ROS_ERROR_STREAM("Connection timeout to DB '" << _db_name << "' while trying to open collection "
                                                                  << _collection_name);
                }
                // Check for empty database
                if (!_conn->isConnected() || !_coll) {
                    ROS_ERROR_STREAM("Connection to DB failed for collection '" << _collection_name << "'.");
                }
                // TODO: return value and effect for this.
            }

            bool remove(uint32_t uid) {
                // value is already reserved
                std::vector<uint32_t>::iterator it = std::find(_registry.begin(), _registry.end(), uid);
                if (it != _registry.end()) _registry.erase(it);

                // value is in db cache
                // remove from cache

                // remove from DB
                QueryPtr query = _coll->createQuery();
                query->append("uid", (int) uid);
                _coll->removeMessages(query);
                return true;
            }

            bool has(int uid) {
                // TODO: doc, no revisa por uids ya registradas
                QueryPtr query = _coll->createQuery();
                query->append("uid", uid);
                try {
                    _coll->findOne(query, true);
                }
                catch (const warehouse_ros::NoMatchingMessageException &exception) {
                    return false;
                }
                return true;
            }

            int count() {
                return _coll->count();
            }

            bool drop_db() {
                _conn->dropDatabase(_db_name);
                _registry.clear();
                // TODO: clear cache
                setup_db();
                return true;
            }

            bool get(uint32_t uid, StreamWithMetadataPtr &stream_ptr) {
                QueryPtr query = _coll->createQuery();
                query->append("uid", (int) uid);
                try {
                    stream_ptr = _coll->findOne(query, false);
                }
                catch (const warehouse_ros::NoMatchingMessageException &exception) {
                    stream_ptr.reset();
                    return false;
                }
                return true;
            }

            bool insert(const StreamType &stream, MetadataPtr metadata) {
                _coll->insert(stream, metadata);
                // todo: insert into cache
                ROS_INFO_STREAM(_log_prefix << "Inserting stream (" << stream.uid << ") into collection "
                                            << "'" << _collection_name << "'. (" << count() << ") entries."
                );
                return true;
            }

            MetadataPtr create_metadata() {
                return _coll->createMetadata();
            }

            bool update(uint32_t uid, const StreamType &stream) {
                ROS_WARN("UPDATE: Method not implemented");
                return false;
            }


            // -----------------------------------------------------------------------------------------------------------------
            // ROS API
            // -----------------------------------------------------------------------------------------------------------------

            bool status_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
                ROS_INFO_STREAM(
                        _log_prefix << "Collection '" << _collection_name << "' has " << count() << " entries.");
                return true;
            }

            bool drop_db_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
                ROS_WARN_STREAM(_log_prefix << "Deleting all entries from collection '" << _collection_name << "'.");
                std_srvs::Empty srv;
                drop_db();
                status_service(srv.request, srv.response);
                return true;
            }

            bool add_service(StreamSrvRequest &req, StreamSrvResponse &res) {

            }

            bool get_service(StreamSrvRequest &req, StreamSrvResponse &res) {
                ROS_INFO_STREAM(_log_prefix << "Retrieving stream (" << req.uid << ") from collection '"
                                            << get_collection_name() << "'");
                StreamWithMetadataPtr stream_ptr;
                if (!get(req.uid, stream_ptr)) {
                    ROS_ERROR_STREAM(_log_prefix << "Stream with uid '" << req.uid << "' not found.");
                    res.succeeded = (uint8_t) false;
                    return true;
                }
                res.msg = *stream_ptr;
                res.succeeded = (uint8_t) true;
                return true;
            }

            bool delete_service(StreamSrvRequest &req, StreamSrvResponse &res) {
                ROS_INFO_STREAM(_log_prefix << "Removing (" << req.uid << ") from collection '" << get_collection_name()
                                            << "'");
                res.succeeded = (uint8_t) remove(req.uid);
                return res.succeeded;
            }


        };

    }
}


#endif //LTM_STREAM_COLLECTION_H
