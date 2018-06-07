#ifndef LTM_PLUGIN_STREAM_COLLECTION_IMPL_HXX
#define LTM_PLUGIN_STREAM_COLLECTION_IMPL_HXX

#include <ltm/db/stream_collection.h>

namespace ltm {
    namespace db {

        template<class StreamType>
        std::string StreamCollectionManager<StreamType>::ltm_get_type() {
            return _type;
        }

        template<class StreamType>
        std::string StreamCollectionManager<StreamType>::ltm_get_collection_name() {
            return _collection_name;
        }

        template<class StreamType>
        void StreamCollectionManager<StreamType>::ltm_setup_db(DBConnectionPtr db_ptr, std::string db_name, std::string collection_name, std::string type) {
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

        template<class StreamType>
        bool StreamCollectionManager<StreamType>::ltm_register_episode(uint32_t uid) {
            ROS_DEBUG_STREAM(_log_prefix << "Registering episode " << uid);

            // subscribe on demand
            bool subscribe = false;
            if (_registry.empty()) subscribe = true;

            // register in cache
            std::vector<uint32_t>::const_iterator it = std::find(_registry.begin(), _registry.end(), uid);
            if (it == _registry.end()) _registry.push_back(uid);

            return subscribe;
        }

        template<class StreamType>
        bool StreamCollectionManager<StreamType>::ltm_unregister_episode(uint32_t uid) {
            ROS_DEBUG_STREAM(_log_prefix << "Unregistering episode " << uid);

            // unregister from cache
            std::vector<uint32_t>::iterator it = std::find(_registry.begin(), _registry.end(), uid);
            if (it != _registry.end()) _registry.erase(it);

            // unsubscribe on demand
            bool unsubscribe = false;
            if (_registry.empty()) unsubscribe = true;
            return unsubscribe;
        }

        template<class StreamType>
        bool StreamCollectionManager<StreamType>::ltm_is_reserved(int uid) {
            // value is in registry
            std::vector<uint32_t>::iterator it = std::find(_registry.begin(), _registry.end(), uid);
            if (it != _registry.end()) return true;

            // TODO: value is in db cache

            // value is already in DB
            return ltm_has(uid);
        }

        template<class StreamType>
        bool StreamCollectionManager<StreamType>::ltm_remove(uint32_t uid) {
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

        template<class StreamType>
        bool StreamCollectionManager<StreamType>::ltm_has(int uid) {
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

        template<class StreamType>
        int StreamCollectionManager<StreamType>::ltm_count() {
            return _coll->count();
        }

        template<class StreamType>
        bool StreamCollectionManager<StreamType>::ltm_drop_db() {
            _conn->dropDatabase(_db_name);
            _registry.clear();
            // TODO: clear cache
            ltm_setup_db(_conn, _db_name, _collection_name, _type);
            return true;
        }

        template<class StreamType>
        bool StreamCollectionManager<StreamType>::ltm_get(uint32_t uid, StreamWithMetadataPtr &stream_ptr) {
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

        template<class StreamType>
        bool StreamCollectionManager<StreamType>::ltm_insert(const StreamType &stream, MetadataPtr metadata) {
            _coll->insert(stream, metadata);
            // todo: insert into cache
            ROS_INFO_STREAM(_log_prefix << "Inserting stream (" << stream.uid << ") into collection "
                                        << "'" << _collection_name << "'. (" << ltm_count() << ") entries."
            );
            return true;
        }

        template<class StreamType>
        MetadataPtr StreamCollectionManager<StreamType>::ltm_create_metadata() {
            return _coll->createMetadata();
        }

        template<class StreamType>
        bool StreamCollectionManager<StreamType>::ltm_update(uint32_t uid, const StreamType &stream) {
            ROS_WARN("UPDATE: Method not implemented");
            return false;
        }

    }
}

#endif //LTM_PLUGIN_STREAM_COLLECTION_IMPL_HXX
