#ifndef LTM_PLUGIN_ENTITY_COLLECTION_IMPL_HXX
#define LTM_PLUGIN_ENTITY_COLLECTION_IMPL_HXX

#include <ltm/db/entity_collection.h>

namespace ltm {
    namespace db {

        template<class EntityType>
        std::string EntityCollectionManager<EntityType>::ltm_get_type() {
            return _type;
        }

        template<class EntityType>
        std::string EntityCollectionManager<EntityType>::ltm_get_collection_name() {
            return _collection_name;
        }

        template<class EntityType>
        std::string EntityCollectionManager<EntityType>::ltm_get_status() {
            std::stringstream ss;
            ss << "Entity '" << _type << "' has (" << ltm_count()
               << ") instances in collection '" << _collection_name
               << "' and (" << ltm_log_count() << "/" << ltm_diff_count() << ") logs.";
            return ss.str();
        }

        template<class EntityType>
        void EntityCollectionManager<EntityType>::ltm_resetup_db() {
            try {
                // host, port, timeout
                _coll = _conn->openCollectionPtr<EntityType>(_db_name, _collection_name);
            }
            catch (const ltm_db::DbConnectException &exception) {
                // Connection timeout
                ROS_ERROR_STREAM("Connection timeout to DB '" << _db_name << "' while trying to open collection "
                                                              << _collection_name);
            }
            try {
                // host, port, timeout
                _diff_coll = _conn->openCollectionPtr<EntityType>(_db_name, _diff_collection_name);
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
        }

        template<class EntityType>
        void EntityCollectionManager<EntityType>::ltm_setup_db(DBConnectionPtr db_ptr, std::string db_name, std::string collection_name, std::string type) {
            _db_name = db_name;
            _collection_name = "entity:" + collection_name;
            _log_collection_name = "entity:" + collection_name + ".meta";
            _diff_collection_name = "entity:" + collection_name + ".trail";
            _type = type;
            _conn = db_ptr;
            this->ltm_resetup_db();
        }

        template<class EntityType>
        bool EntityCollectionManager<EntityType>::ltm_register_episode(uint32_t uid) {
            ROS_DEBUG_STREAM(_log_prefix << "Registering episode " << uid);

            // subscribe on demand
            bool subscribe = false;
            if (_registry.empty()) subscribe = true;

            // register in cache
            std::vector<uint32_t>::const_iterator it = std::find(_registry.begin(), _registry.end(), uid);
            if (it == _registry.end()) _registry.push_back(uid);

            return subscribe;
        }

        template<class EntityType>
        bool EntityCollectionManager<EntityType>::ltm_unregister_episode(uint32_t uid) {
            ROS_DEBUG_STREAM(_log_prefix << "Unregistering episode " << uid);

            // unregister from cache
            std::vector<uint32_t>::iterator it = std::find(_registry.begin(), _registry.end(), uid);
            if (it != _registry.end()) _registry.erase(it);

            // unsubscribe on demand
            bool unsubscribe = false;
            if (_registry.empty()) unsubscribe = true;
            return unsubscribe;
        }

        template<class EntityType>
        bool EntityCollectionManager<EntityType>::ltm_is_reserved(int uid) {
            // value is in registry
            std::vector<uint32_t>::iterator it = std::find(_registry.begin(), _registry.end(), uid);
            if (it != _registry.end()) return true;

            // TODO: value is in db cache

            // value is already in DB
            return ltm_has(uid);
        }

        template<class EntityType>
        void EntityCollectionManager<EntityType>::ltm_get_registry(std::vector<uint32_t> registry) {
            registry = this->_registry;
        }

        template<class EntityType>
        int EntityCollectionManager<EntityType>::ltm_reserve_log_uid() {
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

        template<class EntityType>
        bool EntityCollectionManager<EntityType>::ltm_remove(uint32_t uid) {
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

        template<class EntityType>
        bool EntityCollectionManager<EntityType>::ltm_has(int uid) {
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

        template<class EntityType>
        bool EntityCollectionManager<EntityType>::ltm_log_has(int uid) {
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

        template<class EntityType>
        bool EntityCollectionManager<EntityType>::ltm_diff_has(int uid) {
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

        template<class EntityType>
        int EntityCollectionManager<EntityType>::ltm_count() {
            return _coll->count();
        }

        template<class EntityType>
        int EntityCollectionManager<EntityType>::ltm_log_count() {
            return _log_coll->count();
        }

        template<class EntityType>
        int EntityCollectionManager<EntityType>::ltm_diff_count() {
            return _diff_coll->count();
        }

        template<class EntityType>
        bool EntityCollectionManager<EntityType>::ltm_drop_db() {
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

            _registry.clear();
            _log_uids_cache.clear();
            _reserved_log_uids.clear();
            ltm_resetup_db();
            return true;
        }

        template<class EntityType>
        bool EntityCollectionManager<EntityType>::ltm_get(uint32_t uid, EntityWithMetadataPtr &entity_ptr) {
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

        template<class EntityType>
        int EntityCollectionManager<EntityType>::ltm_get_last_log_uid(uint32_t entity_uid) {
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

        template<class EntityType>
        bool EntityCollectionManager<EntityType>::ltm_insert(const EntityType &entity, MetadataPtr metadata) {
            _coll->insert(entity, metadata);
            // todo: insert into cache
            ROS_INFO_STREAM(_log_prefix << "Inserting entity (" << entity.uid << ") into collection "
                                        << "'" << _collection_name << "'. (" << ltm_count() << ") entries."
            );
            return true;
        }

        template<class EntityType>
        bool EntityCollectionManager<EntityType>::ltm_log_insert(const LogType &log, MetadataPtr metadata) {
            _log_coll->insert(log, metadata);
            // todo: insert into cache
            ROS_DEBUG_STREAM(_log_prefix << "Inserting LOG (" << log.log_uid << ") for entity (" << log.entity_uid
                                        << ") into collection " << "'" << _log_collection_name
                                        << "'. LOG has (" << ltm_log_count() << ") entries."
            );
            return true;
        }

        template<class EntityType>
        bool EntityCollectionManager<EntityType>::ltm_diff_insert(const EntityType &diff, MetadataPtr metadata) {
            _diff_coll->insert(diff, metadata);
            // todo: insert into cache
            ROS_DEBUG_STREAM(_log_prefix << "Inserting LOG DIFF (" << diff.log_uid << ") for entity (" << diff.uid
                                        << ") into collection " << "'" << _diff_collection_name
                                        << "'. LOG DIFF has (" << ltm_diff_count() << ") entries."
            );
            return true;
        }

        template<class EntityType>
        MetadataPtr EntityCollectionManager<EntityType>::ltm_create_metadata() {
            return _coll->createMetadata();
        }

        template<class EntityType>
        bool EntityCollectionManager<EntityType>::ltm_update(uint32_t uid, const EntityType &entity, MetadataPtr metadata) {
            if (!ltm_has(uid)) {
                ltm_insert(entity, metadata);
            }
            ltm_remove(uid);
            _coll->insert(entity, metadata);
            // todo: insert into cache
            ROS_INFO_STREAM(_log_prefix << "Updating entity (" << entity.uid << ") from collection "
                                        << "'" << _collection_name << "'. (" << ltm_count() << ") entries."
            );
            return true;
        }

    }
}

#endif //LTM_PLUGIN_ENTITY_COLLECTION_IMPL_HXX
