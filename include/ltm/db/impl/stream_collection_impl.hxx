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
        std::string StreamCollectionManager<StreamType>::ltm_get_db_name() {
            return _db_name;
        }

        template<class StreamType>
        std::string StreamCollectionManager<StreamType>::ltm_get_status() {
            std::stringstream ss;
            ss << "Stream '" << _type << "' has (" << ltm_count() << ") entries in collection '" << _collection_name << "'.";
            return ss.str();
        }

        template<class StreamType>
        void StreamCollectionManager<StreamType>::ltm_resetup_db(const std::string &db_name) {
            _db_name = db_name;

            try {
                // host, port, timeout
                _coll = _conn->openCollectionPtr<StreamType>(_db_name, _collection_name);
            }
            catch (const ltm_db::DbConnectException &exception) {
                // Connection timeout
                ROS_ERROR_STREAM("Connection timeout to DB '" << _db_name << "' while trying to open collection "
                                                              << _collection_name);
            }
            // Check for empty database
            if (!_conn->isConnected() || !_coll) {
                ROS_ERROR_STREAM("Connection to DB failed for collection '" << _collection_name << "'.");
            }
            // TODO: return value and effect for this.

            _registry.clear();
            // TODO: clear cache
        }

        template<class StreamType>
        void StreamCollectionManager<StreamType>::ltm_setup_db(DBConnectionPtr db_ptr, std::string db_name, std::string collection_name, std::string type) {
            _collection_name = "stream:" + collection_name;
            _type = type;
            _conn = db_ptr;
            this->ltm_resetup_db(db_name);
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
            catch (const ltm_db::NoMatchingMessageException &exception) {
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
            ROS_WARN_STREAM("Dropping database for '" << ltm_get_type() << "'. Collection " << _collection_name << ".");
            QueryPtr query = _coll->createQuery();
            query->appendGT("uid", -1);
            _coll->removeMessages(query);

            ltm_resetup_db(_db_name);
            return true;
        }

        template<class StreamType>
        bool StreamCollectionManager<StreamType>::ltm_get(uint32_t uid, StreamWithMetadataPtr &stream_ptr) {
            QueryPtr query = _coll->createQuery();
            query->append("uid", (int) uid);
            try {
                stream_ptr = _coll->findOne(query, false);
            }
            catch (const ltm_db::NoMatchingMessageException &exception) {
                stream_ptr.reset();
                return false;
            }
            return true;
        }

        template<class StreamType>
        bool StreamCollectionManager<StreamType>::ltm_query(const std::string &json, ltm::QueryServer::Response &res) {
            QueryPtr query = _coll->createQuery();
            std::vector<StreamWithMetadataPtr> result;
            res.episodes.clear();
            res.streams.clear();
            res.entities.clear();

            // generate query and collect documents.
            try {
                query->append(json);
                result = _coll->queryList(query, true);
            } catch (const ltm_db::NoMatchingMessageException &exception) {
                return false;
            } catch (const mongo::exception &ex) {
                ROS_ERROR_STREAM("Error while quering MongoDB for '" << _type << "' streams. " << ex.what());
            }

            // fill uids
            QueryResult qr;
            qr.type = this->ltm_get_type();
            typename std::vector<StreamWithMetadataPtr>::const_iterator it;
            for (it = result.begin(); it != result.end(); ++it) {
                uint32_t uid = (uint32_t) (*it)->lookupInt("uid");
                uint32_t episode_uid = (uint32_t) (*it)->lookupInt("episode_uid");

                qr.uids.push_back(uid);
                res.episodes.push_back(episode_uid);
            }
            res.streams.push_back(qr);
            ROS_INFO_STREAM("Found (" << result.size() << ") matches, with (" << qr.uids.size()
                                      << ") streams and (" << res.episodes.size() << ") episodes.");
            return true;
        }

        template<class StreamType>
        bool StreamCollectionManager<StreamType>::ltm_insert(const StreamType &stream, MetadataPtr metadata) {
            // add common metadata for streams
            metadata->append("uid", (int) stream.meta.uid);
            metadata->append("episode_uid", (int) stream.meta.episode);
            double _start = stream.meta.start.sec + stream.meta.start.nsec * pow10(-9);
            double _end = stream.meta.end.sec + stream.meta.end.nsec * pow10(-9);
            metadata->append("start", _start);
            metadata->append("end", _end);

            // insert
            _coll->insert(stream, metadata);
            // todo: insert into cache
            ROS_INFO_STREAM(_log_prefix << "Inserting stream (" << stream.meta.uid << ") into collection "
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
