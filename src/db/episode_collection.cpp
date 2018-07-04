
#include <ltm/util/util.h>
#include <ltm/db/episode_collection.h>
#include <algorithm>
#include <sstream>

// get random uid
#include <time.h>
#include <limits>
#include <stdlib.h>

namespace ltm {
    namespace db {

        EpisodeCollectionManager::EpisodeCollectionManager(const std::string &name, const std::string &collection, const std::string &host, uint port, float timeout) {
            _db_name = name;
            _db_collection_name = collection;
            _db_host = host;
            _db_port = port;
            _db_timeout = timeout;

            // reserved uids are cleared on startup
            _reserved_uids.clear();
            _db_uids.clear();
            std::srand((uint) time(NULL));
        }

        EpisodeCollectionManager::~EpisodeCollectionManager() {}


        // =================================================================================================================
        // Private API
        // =================================================================================================================
        //


        // =================================================================================================================
        // Public API
        // =================================================================================================================

        std::string EpisodeCollectionManager::to_short_string(const ltm::Episode &episode) {
            std::stringstream ss;
            ss << "<"
               << "uid:" << episode.uid << ", "
               << "type:" << (int) episode.type << ", "
               << "emotion:" << episode.relevance.emotional.emotion
               << ">";
            return ss.str();
        }

        void EpisodeCollectionManager::setup() {
            // setup DB
            try {
                // host, port, timeout
                _conn.reset(new ltm_db_mongo::MongoDatabaseConnection());
                _conn->setParams(_db_host, _db_port, _db_timeout);
                _conn->connect();
                _coll = _conn->openCollectionPtr<Episode>(_db_name, _db_collection_name);
                EpisodeMetadataBuilder::setup(_coll);
            }
            catch (const ltm_db::DbConnectException &exception) {
                // Connection timeout
                ROS_ERROR_STREAM("Connection timeout to DB '" << _db_name << "'.");
                ros::shutdown();
                exit(1);
            }
            // Check for empty database
            if (!_conn->isConnected() || !_coll) {
                ROS_ERROR_STREAM("Connection to DB failed for collection '" << _db_collection_name << "'.");
                ros::shutdown();
                exit(1);
            }
        }

        // -----------------------------------------------------------------------------------------------------------------
        // CRUD methods
        // -----------------------------------------------------------------------------------------------------------------

        bool EpisodeCollectionManager::insert(const ltm::Episode &episode) {
            // insert into DB
            _coll->insert(episode, make_metadata(episode));

            // insert into cache
            _db_uids.insert(episode.uid);
            return true;
        }

        bool EpisodeCollectionManager::query(const std::string &json, ltm::QueryServer::Response &res) {
            std::vector<EpisodeWithMetadataPtr> result;
            res.episodes.clear();
            res.entities.clear();
            res.streams.clear();

            // generate query and collect documents.
            try {
                QueryPtr query = _coll->createQuery();
                query->append(json);
                // TODO: solve this only using metadata!
                // result = _coll->queryList(query, true);
                result = _coll->queryList(query, false);
            } catch (const ltm_db::NoMatchingMessageException &exception) {
                return false;
            } catch (const mongo::exception &ex) {
                ROS_ERROR_STREAM("Error while quering MongoDB for episodes. " << ex.what());
            }

            // Retrieve uids
            int s_cnt = 0, e_cnt = 0;
            std::map<std::string, ltm::QueryResult> stream_r;
            std::map<std::string, ltm::QueryResult> entity_r;
            std::map<std::string, ltm::QueryResult>::iterator q_it;
            std::vector<EpisodeWithMetadataPtr>::const_iterator it;
            for (it = result.begin(); it != result.end(); ++it) {

                // fill episode uids
                res.episodes.push_back((uint32_t)(*it)->lookupInt("uid"));

                // retrieve stream QueryResults
                std::vector<ltm::StreamRegister>::const_iterator s_cit;
                for (s_cit = (*it)->what.streams.begin(); s_cit != (*it)->what.streams.end(); ++s_cit) {

                    // create QueryResult if not exists
                    q_it = stream_r.find(s_cit->type);
                    if (q_it == stream_r.end()) {
                        ltm::QueryResult qr;
                        qr.type = s_cit->type;
                        qr.uids.push_back(s_cit->uid);
                        stream_r[s_cit->type] = qr;
                        s_cnt++;
                        continue;
                    }

                    // append unique uid
                    std::vector<uint32_t>::const_iterator u_it;
                    u_it = std::find(q_it->second.uids.begin(), q_it->second.uids.end(), s_cit->uid);
                    if (u_it == q_it->second.uids.end()) {
                        q_it->second.uids.push_back(s_cit->uid);
                        s_cnt++;
                    }
                }

                // retrieve entity QueryResults
                std::vector<ltm::EntityRegister>::const_iterator e_cit;
                for (e_cit = (*it)->what.entities.begin(); e_cit != (*it)->what.entities.end(); ++e_cit) {

                    // create QueryResult if not exists
                    q_it = entity_r.find(e_cit->type);
                    if (q_it == entity_r.end()) {
                        ltm::QueryResult qr;
                        qr.type = e_cit->type;
                        qr.uids.push_back(e_cit->uid);
                        entity_r[e_cit->type] = qr;
                        e_cnt++;
                        continue;
                    }

                    // append unique uid
                    std::vector<uint32_t>::const_iterator u_it;
                    u_it = std::find(q_it->second.uids.begin(), q_it->second.uids.end(), e_cit->uid);
                    if (u_it == q_it->second.uids.end()) {
                        q_it->second.uids.push_back(e_cit->uid);
                        e_cnt++;
                    }
                }

            }

            // Fill stream/entity QueryResults
            for (q_it = stream_r.begin(); q_it != stream_r.end(); ++q_it) {
                res.streams.push_back(q_it->second);
            }
            for (q_it = entity_r.begin(); q_it != entity_r.end(); ++q_it) {
                res.entities.push_back(q_it->second);
            }

            ROS_INFO_STREAM("Found (" << result.size() << ") matches, with (" << s_cnt << ") instances of ("
                                      << res.streams.size() << ") streams, and (" << e_cnt << ") instances of ("
                                      << res.entities.size() << ") entities.");
            return true;
        }

        bool EpisodeCollectionManager::get(int uid, EpisodeWithMetadataPtr &episode_ptr) {
            QueryPtr query = _coll->createQuery();
            query->append("uid", uid);
            try {
                episode_ptr = _coll->findOne(query, false);
            }
            catch (const ltm_db::NoMatchingMessageException &exception) {
                episode_ptr.reset();
                return false;
            }
            return true;
        }

        bool EpisodeCollectionManager::update(const ltm::Episode &episode) {
            ROS_WARN("UPDATE: Method not implemented");
            return false;
        }

        bool EpisodeCollectionManager::remove(int uid) {
            std::set<int>::iterator it;

            // value is already reserved
            it = _reserved_uids.find(uid);
            if (it != _reserved_uids.end()) _reserved_uids.erase(it);

            // value is in db cache
            it = _db_uids.find(uid);
            if (it != _db_uids.end()) _db_uids.erase(it);

            // remove from DB
            QueryPtr query = _coll->createQuery();
            query->append("uid", uid);
            _coll->removeMessages(query);
            return true;
        }


        // -----------------------------------------------------------------------------------------------------------------
        // Other Queries
        // -----------------------------------------------------------------------------------------------------------------

        int EpisodeCollectionManager::count() {
            return _coll->count();
        }

        int EpisodeCollectionManager::reserve_uid() {
            // TODO: find better way, maybe select the max_uid+1
            // Returns a pseudo-random integral number in the range between 0 and RAND_MAX.
            uint32_t max_int32 = std::numeric_limits<uint32_t>::max();
            int max_int = std::numeric_limits<int>::max();
            int upper_bound = max_int32 > max_int ? max_int : max_int32;

            int db_size = count();
            int reserved_ids = (int) _reserved_uids.size();
            if (upper_bound <= db_size + reserved_ids) {
                ROS_WARN_STREAM(
                        "There aren't any available uids. Max entries: "
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
                it = _reserved_uids.find(value);
                if (it != _reserved_uids.end()) continue;

                // value is in db cache
                it = _db_uids.find(value);
                if (it != _db_uids.end()) continue;

                // value is already in DB
                if (has(value)) {
                    _db_uids.insert(value);
                    continue;
                }
                break;
            }
            _reserved_uids.insert(value);
            return value;
        }

        bool EpisodeCollectionManager::is_reserved(int uid) {
            std::set<int>::iterator it;

            // value is already reserved
            it = _reserved_uids.find(uid);
            if (it != _reserved_uids.end()) return true;

            // value is in db cache
            it = _db_uids.find(uid);
            if (it != _db_uids.end()) return true;

            // value is already in DB
            return has(uid);
        }

        bool EpisodeCollectionManager::has(int uid) {
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

        bool EpisodeCollectionManager::drop_db() {
            _conn->dropDatabase(_db_name);
            _reserved_uids.clear();
            _db_uids.clear();
            setup();
            return true;
        }

        bool EpisodeCollectionManager::switch_db(const std::string &db_name) {
            _db_name = db_name;
            _reserved_uids.clear();
            _db_uids.clear();
            setup();
            return true;
        }

        bool EpisodeCollectionManager::update_tree(int uid) {
            if (!has(uid)) {
                return false;
            }
            Episode root_ptr;
            return update_tree_node(uid, root_ptr);
        }

        bool EpisodeCollectionManager::update_tree_node(int uid, Episode &updated_episode) {
            ROS_DEBUG_STREAM(" - updating node: " << uid);
            EpisodeWithMetadataPtr ep_ptr;

            // get episode
            if (!get(uid, ep_ptr)) {
                throw ltm_db::NoMatchingMessageException("episode not found");
            }
            updated_episode = *ep_ptr;

            // is leaf
            if (ep_ptr->type == Episode::LEAF) {
                ROS_DEBUG_STREAM(" ---> node " << uid << " is a leaf, will not update it.");
                return true;
            }

            // init fields
            EpisodeUpdateHelper helper;
            this->update_tree_init(updated_episode, helper);

            // process children
            bool result = true;
            Episode child;
            std::vector<uint32_t>::const_iterator c_it;
            for (c_it = ep_ptr->children_ids.begin(); c_it != ep_ptr->children_ids.end(); ++c_it) {
                // recurse to children
                result = result && update_tree_node(*c_it, child);

                // update fields
                result = result && this->update_from_child(updated_episode, child, helper);
            }
            // finish fields
            result = result && this->update_tree_last(updated_episode, helper);

            // save updated episode
            // TODO: update instead of remove/insert
            result = result && remove(uid);
            result = result && insert(updated_episode);
            ROS_DEBUG_STREAM(" -> node updated: " << uid);
            ROS_ERROR_STREAM_COND(!result, "UPDATE TREE: An error occurred while updating episode (" << uid << ")");
            return result;
        }

        bool EpisodeCollectionManager::update_from_children(ltm::Episode &episode) {

            // init fields
            EpisodeUpdateHelper helper;
            this->update_tree_init(episode, helper);

            // iterate over children
            std::vector<uint32_t> bad_children;
            std::vector<uint32_t>::const_iterator it;
            bool result = true;
            for (it = episode.children_ids.begin(); it != episode.children_ids.end(); ++it) {
                EpisodeWithMetadataPtr child_ptr;
                if (!this->get(*it, child_ptr)) {
                    bad_children.push_back(*it);
                    continue;
                }
                result = result && this->update_from_child(episode, *child_ptr, helper);
            }

            // end fields
            result = result && this->update_tree_last(episode, helper);

            // TODO: remove bad children from children_ids
            return result;
        }

    }
}