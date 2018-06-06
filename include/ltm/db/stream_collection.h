#ifndef LTM_STREAM_COLLECTION_H
#define LTM_STREAM_COLLECTION_H

#include <set>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ltm/db/types.h>
#include <ltm/Episode.h>

namespace ltm {

    namespace db {

        template<class StreamType>
        class StreamCollectionManager {
        private:
            typedef warehouse_ros::MessageCollection<StreamType> StreamCollection;
            typedef boost::shared_ptr<StreamCollection> StreamCollectionPtr;

            typedef warehouse_ros::MessageWithMetadata<StreamType> StreamWithMetadata;
            typedef boost::shared_ptr<const StreamWithMetadata> StreamWithMetadataPtr;

            // database connection
            StreamCollectionPtr _coll;
            DBConnectionPtr _conn;

            // database parameters
            std::string _collection_name;
            std::string _db_name;
            std::string _type;

            // control
            std::vector<uint32_t> _registry;

        public:
            std::string _log_prefix;


            std::string ltm_get_type() {
                return _type;
            }

            std::string ltm_get_collection_name() {
                return _collection_name;
            }

            void ltm_setup_db(DBConnectionPtr db_ptr, std::string db_name, std::string collection_name, std::string type) {
                _db_name = db_name;
                _collection_name = collection_name;
                _type = type;
                _conn = db_ptr;
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

            bool ltm_register_episode(uint32_t uid) {
                ROS_DEBUG_STREAM(_log_prefix << "Registering episode " << uid);

                // subscribe on demand
                bool subscribe = false;
                if (_registry.empty()) subscribe = true;

                // register in cache
                std::vector<uint32_t>::const_iterator it = std::find(_registry.begin(), _registry.end(), uid);
                if (it == _registry.end()) _registry.push_back(uid);

                return subscribe;
            }

            bool ltm_unregister_episode(uint32_t uid) {
                ROS_DEBUG_STREAM(_log_prefix << "Unregistering episode " << uid);

                // unregister from cache
                std::vector<uint32_t>::iterator it = std::find(_registry.begin(), _registry.end(), uid);
                if (it != _registry.end()) _registry.erase(it);

                // unsubscribe on demand
                bool unsubscribe = false;
                if (_registry.empty()) unsubscribe = true;
                return unsubscribe;
            }

            bool ltm_is_reserved(int uid) {
                // value is in registry
                std::vector<uint32_t>::iterator it = std::find(_registry.begin(), _registry.end(), uid);
                if (it != _registry.end()) return true;

                // TODO: value is in db cache

                // value is already in DB
                return ltm_has(uid);
            }

            bool ltm_remove(uint32_t uid) {
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

            bool ltm_has(int uid) {
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

            int ltm_count() {
                return _coll->count();
            }

            bool ltm_drop_db() {
                _conn->dropDatabase(_db_name);
                _registry.clear();
                // TODO: clear cache
                ltm_setup_db(_conn, _db_name, _collection_name, _type);
                return true;
            }

            bool ltm_get(uint32_t uid, StreamWithMetadataPtr &stream_ptr) {
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

            bool ltm_insert(const StreamType &stream, MetadataPtr metadata) {
                _coll->insert(stream, metadata);
                // todo: insert into cache
                ROS_INFO_STREAM(_log_prefix << "Inserting stream (" << stream.uid << ") into collection "
                                            << "'" << _collection_name << "'. (" << ltm_count() << ") entries."
                );
                return true;
            }

            MetadataPtr ltm_create_metadata() {
                return _coll->createMetadata();
            }

            bool ltm_update(uint32_t uid, const StreamType &stream) {
                ROS_WARN("UPDATE: Method not implemented");
                return false;
            }

        };

    }
}


#endif //LTM_STREAM_COLLECTION_H
