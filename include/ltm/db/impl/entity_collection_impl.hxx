#ifndef LTM_PLUGIN_ENTITY_COLLECTION_IMPL_HXX
#define LTM_PLUGIN_ENTITY_COLLECTION_IMPL_HXX

#include <ltm/db/entity_collection.h>
#include <ltm/util/util.h>

namespace ltm {
    namespace db {

        template<class EntityMsg>
        std::string EntityCollectionManager<EntityMsg>::ltm_get_type() {
            return _type;
        }

        template<class EntityMsg>
        std::string EntityCollectionManager<EntityMsg>::ltm_get_collection_name() {
            return _collection_name;
        }

        template<class EntityMsg>
        std::string EntityCollectionManager<EntityMsg>::ltm_get_db_name() {
            return _db_name;
        }

        template<class EntityMsg>
        std::string EntityCollectionManager<EntityMsg>::ltm_get_status() {
            std::stringstream ss;
            ss << "Entity '" << _type << "' has (" << ltm_count()
               << ") instances in collection '" << _collection_name
               << "' and (" << ltm_log_count() << "/" << ltm_diff_count() << ") logs.";
            return ss.str();
        }

        template<class EntityMsg>
        void EntityCollectionManager<EntityMsg>::ltm_resetup_db(const std::string &db_name) {
            _db_name = db_name;

            try {
                // host, port, timeout
                _coll = _conn->openCollectionPtr<EntityMsg>(_db_name, _collection_name);
            }
            catch (const ltm_db::DbConnectException &exception) {
                // Connection timeout
                ROS_ERROR_STREAM("Connection timeout to DB '" << _db_name << "' while trying to open collection "
                                                              << _collection_name);
            }
            try {
                // host, port, timeout
                _diff_coll = _conn->openCollectionPtr<EntityMsg>(_db_name, _diff_collection_name);
            }
            catch (const ltm_db::DbConnectException &exception) {
                // Connection timeout
                ROS_ERROR_STREAM("Connection timeout to DB '" << _db_name << "' while trying to open collection "
                                                              << _diff_collection_name);
            }
            try {
                // host, port, timeout
                _log_coll = _conn->openCollectionPtr<LogType>(_db_name, _log_collection_name);
            }
            catch (const ltm_db::DbConnectException &exception) {
                // Connection timeout
                ROS_ERROR_STREAM("Connection timeout to DB '" << _db_name << "' while trying to open collection "
                                                              << _log_collection_name);
            }
            // Check for empty database
            if (!_conn->isConnected() || !_coll) {
                ROS_ERROR_STREAM("Connection to DB failed for collection '" << _collection_name << "'.");
            }
            // TODO: return value and effect for this.

            _registry.clear();
            _log_uids_cache.clear();
            _reserved_log_uids.clear();
        }

        template<class EntityMsg>
        void EntityCollectionManager<EntityMsg>::ltm_setup_db(DBConnectionPtr db_ptr, std::string db_name, std::string collection_name, std::string type) {
            _collection_name = "entity:" + collection_name;
            _log_collection_name = "entity:" + collection_name + ".meta";
            _diff_collection_name = "entity:" + collection_name + ".trail";
            _type = type;
            _conn = db_ptr;
            this->ltm_resetup_db(db_name);
        }

        template<class EntityMsg>
        bool EntityCollectionManager<EntityMsg>::ltm_register_episode(uint32_t uid) {
            // subscribe on demand
            bool subscribe = false;
            if (_registry.empty()) subscribe = true;

            // register in cache
            std::vector<uint32_t>::const_iterator it = std::find(_registry.begin(), _registry.end(), uid);
            if (it == _registry.end()) _registry.push_back(uid);

            return subscribe;
        }

        template<class EntityMsg>
        bool EntityCollectionManager<EntityMsg>::ltm_unregister_episode(uint32_t uid) {
            // unregister from cache
            std::vector<uint32_t>::iterator it = std::find(_registry.begin(), _registry.end(), uid);
            if (it != _registry.end()) _registry.erase(it);

            // unsubscribe on demand
            bool unsubscribe = false;
            if (_registry.empty()) unsubscribe = true;
            return unsubscribe;
        }

        template<class EntityMsg>
        bool EntityCollectionManager<EntityMsg>::ltm_is_reserved(int uid) {
            // value is in registry
            std::vector<uint32_t>::iterator it = std::find(_registry.begin(), _registry.end(), uid);
            if (it != _registry.end()) return true;

            // TODO: value is in db cache

            // value is already in DB
            return ltm_has(uid);
        }

        template<class EntityMsg>
        void EntityCollectionManager<EntityMsg>::ltm_get_registry(std::vector<uint32_t> &registry) {
            registry = this->_registry;
        }

        template<class EntityMsg>
        int EntityCollectionManager<EntityMsg>::ltm_reserve_log_uid() {
            // TODO: find better way, maybe select the max_uid+1
            // Returns a pseudo-random integral number in the range between 0 and RAND_MAX.
            uint32_t max_int32 = std::numeric_limits<uint32_t>::max();
            int max_int = std::numeric_limits<int>::max();
            int upper_bound = max_int32 > max_int ? max_int : max_int32;

            int db_size = ltm_log_count();
            int reserved_ids = (int) _reserved_log_uids.size();
            if (upper_bound <= db_size + reserved_ids) {
                ROS_WARN_STREAM(
                        "There aren't any available LOG uids. Max entries: "
                                << upper_bound
                                << ". LTM DB has (" << db_size << ") entries."
                                << " There are (" << reserved_ids << ") reserved uids."
                );
                return -1;
            }

            int value;
            std::set<int>::iterator it;
            while (true) {
                value = rand() % upper_bound;

                // value is already reserved
                it = _reserved_log_uids.find(value);
                if (it != _reserved_log_uids.end()) continue;

                // value is in db cache
                it = _log_uids_cache.find(value);
                if (it != _log_uids_cache.end()) continue;

                // value is already in DB
                if (ltm_log_has(value)) {
                    _log_uids_cache.insert(value);
                    continue;
                }
                break;
            }
            _reserved_log_uids.insert(value);
            return value;
        }

        template<class EntityMsg>
        bool EntityCollectionManager<EntityMsg>::ltm_remove(uint32_t uid) {
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

        template<class EntityMsg>
        bool EntityCollectionManager<EntityMsg>::ltm_has(int uid) {
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

        template<class EntityMsg>
        bool EntityCollectionManager<EntityMsg>::ltm_log_has(int uid) {
            // TODO: doc, no revisa por uids ya registradas
            QueryPtr query = _log_coll->createQuery();
            query->append("log_uid", uid);
            try {
                _coll->findOne(query, true);
            }
            catch (const ltm_db::NoMatchingMessageException &exception) {
                return false;
            }
            return true;
        }

        template<class EntityMsg>
        bool EntityCollectionManager<EntityMsg>::ltm_diff_has(int uid) {
            // TODO: doc, no revisa por uids ya registradas
            QueryPtr query = _diff_coll->createQuery();
            query->append("log_uid", uid);
            try {
                _coll->findOne(query, true);
            }
            catch (const ltm_db::NoMatchingMessageException &exception) {
                return false;
            }
            return true;
        }

        template<class EntityMsg>
        int EntityCollectionManager<EntityMsg>::ltm_count() {
            return _coll->count();
        }

        template<class EntityMsg>
        int EntityCollectionManager<EntityMsg>::ltm_log_count() {
            return _log_coll->count();
        }

        template<class EntityMsg>
        int EntityCollectionManager<EntityMsg>::ltm_diff_count() {
            return _diff_coll->count();
        }

        template<class EntityMsg>
        bool EntityCollectionManager<EntityMsg>::ltm_drop_db() {
            ROS_WARN_STREAM("Dropping database for '" << ltm_get_type() << "'. Collections: " << _collection_name << " and {.meta, .trail}.");
            QueryPtr query = _coll->createQuery();
            query->appendGT("uid", -1);
            _coll->removeMessages(query);

            QueryPtr query_log = _log_coll->createQuery();
            query->appendGT("log_uid", -1);
            _log_coll->removeMessages(query_log);

            QueryPtr query_diff = _diff_coll->createQuery();
            query->appendGT("log_uid", -1);
            _diff_coll->removeMessages(query_diff);

            ltm_resetup_db(_db_name);
            return true;
        }

        template<class EntityMsg>
        bool EntityCollectionManager<EntityMsg>::ltm_get(uint32_t uid, EntityWithMetadataPtr &entity_ptr) {
            QueryPtr query = _coll->createQuery();
            query->append("uid", (int) uid);
            try {
                entity_ptr = _coll->findOne(query, false);
            }
            catch (const ltm_db::NoMatchingMessageException &exception) {
                entity_ptr.reset();
                return false;
            }
            return true;
        }

        template<class EntityMsg>
        int EntityCollectionManager<EntityMsg>::ltm_get_last_log_uid(uint32_t entity_uid) {
            QueryPtr query = _coll->createQuery();
            EntityWithMetadataPtr entity_ptr;
            query->append("uid", (int) entity_uid);
            try {
                entity_ptr = _coll->findOne(query, true);
            }
            catch (const ltm_db::NoMatchingMessageException &exception) {
                entity_ptr.reset();
                return 0;
            }
            return entity_ptr->lookupInt("log_uid");
        }

        template<class EntityMsg>
        bool EntityCollectionManager<EntityMsg>::ltm_query_log(const std::string &json, ltm::QueryServer::Response &res) {
            // generate query and collect documents.
            std::vector<LogWithMetadataPtr> result;
            try {
                QueryPtr query = _log_coll->createQuery();
                query->append(json);
                result = _log_coll->queryList(query, true);
            } catch (const ltm_db::NoMatchingMessageException &exception) {
                return false;
            } catch (const mongo::exception &ex) {
                ROS_ERROR_STREAM("Error while quering MongoDB for entries in '" << _log_collection_name << "' collection. " << ex.what());
            }

            // fill uids
            ltm::QueryResult qr, qre;
            qr.type = this->ltm_get_type();
            qre.type = this->ltm_get_type();
            typename std::vector<LogWithMetadataPtr>::const_iterator it;
            for (it = result.begin(); it != result.end(); ++it) {
                uint32_t uid = (uint32_t) (*it)->lookupInt("log_uid");
                uint32_t e_uid = (uint32_t) (*it)->lookupInt("entity_uid");

                // do not repeat entities
                if (std::find(qre.uids.begin(), qre.uids.end(), e_uid) == qre.uids.end()) {
                    qre.uids.push_back(e_uid);
                }
                qr.uids.push_back(uid);

                std::vector<uint32_t> episodes;
                (*it)->lookupUInt32Array("episode_uids", episodes);
                ltm::util::uid_vector_merge(res.episodes, episodes);
            }
            res.entities.push_back(qre);
            res.entities_trail.push_back(qr);
            ROS_INFO_STREAM("Found (" << result.size() << ") matches, with (" << qr.uids.size()
                                      << ") entity logs.");
            return true;
        }

        template<class EntityMsg>
        bool EntityCollectionManager<EntityMsg>::ltm_query_actual(const std::string &json, ltm::QueryServer::Response &res) {
            // generate query and collect documents.
            std::vector<EntityWithMetadataPtr> result;
            try {
                QueryPtr query = _coll->createQuery();
                query->append(json);
                result = _coll->queryList(query, true);
            } catch (const ltm_db::NoMatchingMessageException &exception) {
                return false;
            } catch (const mongo::exception &ex) {
                ROS_ERROR_STREAM("Error while quering MongoDB for '" << _type << "' entities. " << ex.what());
            }

            // fill uids
            ltm::QueryResult qr;
            qr.type = this->ltm_get_type();
            typename std::vector<EntityWithMetadataPtr>::const_iterator it;
            for (it = result.begin(); it != result.end(); ++it) {
                uint32_t uid = (uint32_t) (*it)->lookupInt("uid");
                qr.uids.push_back(uid);
            }
            res.entities.push_back(qr);
            ROS_INFO_STREAM("Found (" << result.size() << ") matches, with (" << qr.uids.size()
                                      << ") entities.");
            return true;
        }

        template<class EntityMsg>
        bool EntityCollectionManager<EntityMsg>::ltm_query(const std::string &json, ltm::QueryServer::Response &res, bool trail) {
            res.episodes.clear();
            res.streams.clear();
            res.entities.clear();
            res.entities_trail.clear();
            if (trail) return ltm_query_log(json, res);
            return ltm_query_actual(json, res);
        }

        template<class EntityMsg>
        bool EntityCollectionManager<EntityMsg>::ltm_insert(const EntityMsg &entity) {
            // insert
            _coll->insert(entity, this->make_metadata(entity));
            // todo: insert into cache
            ROS_INFO_STREAM(_log_prefix << "Inserting entity (" << entity.meta.uid << ") into collection "
                                        << "'" << _collection_name << "'. (" << ltm_count() << ") entries."
            );
            return true;
        }

        template<class EntityMsg>
        bool EntityCollectionManager<EntityMsg>::ltm_log_insert(const LogType &log) {
            _log_coll->insert(log, ltm_make_log_metadata(log));
            // todo: insert into cache
            ROS_DEBUG_STREAM(_log_prefix << "Inserting LOG (" << log.log_uid << ") for entity (" << log.entity_uid
                                        << ") into collection " << "'" << _log_collection_name
                                        << "'. LOG has (" << ltm_log_count() << ") entries."
            );
            return true;
        }

        template<class EntityMsg>
        bool EntityCollectionManager<EntityMsg>::ltm_diff_insert(const EntityMsg &diff) {
            _diff_coll->insert(diff, this->make_metadata(diff));
            // todo: insert into cache
            ROS_DEBUG_STREAM(_log_prefix << "Inserting LOG DIFF (" << diff.meta.log_uid << ") for entity (" << diff.meta.uid
                                        << ") into collection " << "'" << _diff_collection_name
                                        << "'. LOG DIFF has (" << ltm_diff_count() << ") entries."
            );
            return true;
        }

        template<class EntityMsg>
        MetadataPtr EntityCollectionManager<EntityMsg>::ltm_create_metadata(const EntityMsg &entity) {
            MetadataPtr metadata = _coll->createMetadata();
            metadata->append("uid", (int) entity.meta.uid);
            metadata->append("log_uid", (int) entity.meta.log_uid);
            return metadata;
        }

        template<class EntityMsg>
        MetadataPtr EntityCollectionManager<EntityMsg>::ltm_make_log_metadata(const ltm::EntityLog &log) {
            MetadataPtr meta = _log_coll->createMetadata();

            // KEYS
            meta->append("entity_uid", (int) log.entity_uid);
            meta->append("log_uid", (int) log.log_uid);

            // WHO
            meta->append("episode_uids", log.episode_uids);

            // WHEN
            double timestamp = log.timestamp.sec + log.timestamp.nsec * pow10(-9);
            meta->append("timestamp", timestamp);
            meta->append("prev_log", (int) log.prev_log);
            meta->append("next_log", (int) log.next_log);

            // WHAT
            meta->append("new_f", log.new_f);
            meta->append("updated_f", log.updated_f);
            meta->append("removed_f", log.removed_f);

            return meta;
        }

        template<class EntityMsg>
        bool EntityCollectionManager<EntityMsg>::ltm_update(uint32_t uid, const EntityMsg &entity) {
            if (!ltm_has(uid)) {
                ltm_insert(entity);
            }
            ltm_remove(uid);
            _coll->insert(entity, this->make_metadata(entity));
            // todo: insert into cache
            ROS_INFO_STREAM(_log_prefix << "Updating entity (" << entity.meta.uid << ") from collection "
                                        << "'" << _collection_name << "'. (" << ltm_count() << ") entries."
            );
            return true;
        }

    }
}

#endif //LTM_PLUGIN_ENTITY_COLLECTION_IMPL_HXX
