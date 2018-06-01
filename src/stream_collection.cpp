#include <ltm/db/stream_collection.h>

namespace ltm {
    namespace db {

//        template <class T>
//        StreamCollectionManager<T>::StreamCollectionManager(DBConnectionPtr ptr, std::string db_name,
//                                                         std::string collection_name, std::string type,
//                                                         std::string log_prefix) {
//            _conn = ptr;
//            _db_name = db_name;
//            _collection_name = collection_name;
//            _type = type;
//            _log_prefix = log_prefix;
//        }
//
//
//        template <class T>
//        std::string StreamCollectionManager<T>::get_type() {
//            return _type;
//        }
//
//        template <class T>
//        std::string StreamCollectionManager<T>::get_collection_name() {
//            return _collection_name;
//        }
//
//        template <class T>
//        void StreamCollectionManager<T>::register_episode(uint32_t uid) {
//            ROS_DEBUG_STREAM(_log_prefix << "LTM Image Stream plugin: registering episode " << uid);
//
//            // subscribe on demand
//            if (_registry.empty()) subscribe();
//
//            // register in cache
//            std::vector<uint32_t>::const_iterator it = std::find(_registry.begin(), _registry.end(), uid);
//            if (it == _registry.end()) _registry.push_back(uid);
//        }
//
//        template <class T>
//        void StreamCollectionManager<T>::unregister_episode(uint32_t uid) {
//            ROS_DEBUG_STREAM(_log_prefix << "LTM Image Stream plugin: unregistering episode " << uid);
//
//            // unregister from cache
//            std::vector<uint32_t>::iterator it = std::find(_registry.begin(), _registry.end(), uid);
//            if (it != _registry.end()) _registry.erase(it);
//
//            // unsubscribe on demand
//            if (_registry.empty()) unsubscribe();
//        }
//
//        template <class T>
//        bool StreamCollectionManager<T>::is_reserved(int uid) {
//            // value is in registry
//            std::vector<uint32_t>::iterator it = std::find(_registry.begin(), _registry.end(), uid);
//            if (it != _registry.end()) return true;
//
//            // TODO: value is in db cache
//
//            // value is already in DB
//            return has(uid);
//        }
//
//        template <class T>
//        void StreamCollectionManager<T>::setup_db() {
//            try {
//                // host, port, timeout
//                _coll = _conn->openCollectionPtr<T>(_db_name, _collection_name);
//            }
//            catch (const warehouse_ros::DbConnectException& exception) {
//                // Connection timeout
//                ROS_ERROR_STREAM("Connection timeout to DB '" << _db_name << "' while trying to open collection " << _collection_name);
//            }
//            // Check for empty database
//            if (!_conn->isConnected() || !_coll) {
//                ROS_ERROR_STREAM("Connection to DB failed for collection '" << _collection_name << "'.");
//            }
//            // TODO: return value and effect for this.
//        }
//
//        template <class T>
//        bool StreamCollectionManager<T>::remove(uint32_t uid) {
//            // value is already reserved
//            std::vector<uint32_t>::iterator it = std::find(_registry.begin(), _registry.end(), uid);
//            if (it != _registry.end()) _registry.erase(it);
//
//            // value is in db cache
//            // remove from cache
//
//            // remove from DB
//            QueryPtr query = _coll->createQuery();
//            query->append("uid", (int)uid);
//            _coll->removeMessages(query);
//            return true;
//        }
//
//        template <class T>
//        bool StreamCollectionManager<T>::has(int uid) {
//            // TODO: doc, no revisa por uids ya registradas
//            QueryPtr query = _coll->createQuery();
//            query->append("uid", uid);
//            try {
//                _coll->findOne(query, true);
//            }
//            catch (const warehouse_ros::NoMatchingMessageException& exception) {
//                return false;
//            }
//            return true;
//        }
//
//        template <class T>
//        int StreamCollectionManager<T>::count() {
//            return _coll->count();
//        }
//
//        template <class T>
//        bool StreamCollectionManager<T>::drop_db() {
//            _conn->dropDatabase(_db_name);
//            _registry.clear();
//            // TODO: clear cache
//            setup_db();
//            return true;
//        }
//
//        template <class T>
//        bool StreamCollectionManager<T>::get(uint32_t uid, StreamWithMetadataPtr &stream_ptr) {
//            QueryPtr query = _coll->createQuery();
//            query->append("uid", (int)uid);
//            try {
//                stream_ptr = _coll->findOne(query, false);
//            }
//            catch (const warehouse_ros::NoMatchingMessageException& exception) {
//                stream_ptr.reset();
//                return false;
//            }
//            return true;
//        }
//
//        template <class T>
//        bool StreamCollectionManager<T>::insert(const T &stream, MetadataPtr metadata) {
//            _coll->insert(stream, metadata);
//            // todo: insert into cache
//            ROS_INFO_STREAM(_log_prefix << "Inserting stream (" << stream.uid << ") into collection "
//                            << "'" << _collection_name << "'. (" << count() << ") entries."
//            );
//            return true;
//        }
//
//        template <class T>
//        MetadataPtr StreamCollectionManager<T>::create_metadata() {
//            return _coll->createMetadata();
//        }
//
//        template <class T>
//        void StreamCollectionManager<T>::subscribe() {
//
//        }
//
//        template <class T>
//        void StreamCollectionManager<T>::unsubscribe() {
//
//        }

    }
}