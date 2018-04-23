#include <ltm/db_manager.h>

namespace ltm {

    Manager::Manager(const std::string &name, const std::string &collection, const std::string &host, uint port,
                     float timeout) {
        _db_name = name;
        _db_collection_name = collection;
        _db_host = host;
        _db_port = port;
        _db_timeout = timeout;
    }

    Manager::~Manager() {}

    void Manager::setup() {
        // setup DB
        try {
            // host, port, timeout
            _conn.setParams(_db_host, _db_port, _db_timeout);
            _conn.connect();
            _coll = _conn.openCollectionPtr<Episode>(_db_name, _db_collection_name);
        }
        catch (const warehouse_ros::DbConnectException& exception) {
            // Connection timeout
            ROS_ERROR_STREAM("Connection timeout to DB '" << _db_name << "'.");
            ros::shutdown();
            exit(1);
        }
        // Check for empty database
        if (!_conn.isConnected() || !_coll) {
            ROS_ERROR_STREAM("Connection to DB failed for collection '" << _db_collection_name << "'.");
            ros::shutdown();
            exit(1);
        }
    }

    bool Manager::remove_by_uid(int uid) {
        QueryPtr query = _coll->createQuery();
        query->append("uid", uid);
        _coll->removeMessages(query);
        return true;
    }

    int Manager::count() {
        return _coll->count();
    }

    MetadataPtr Manager::makeMetadata(EpisodeCollectionPtr coll_ptr, const Episode& episode) {
//        // Create metadata, this data is used for queries
//        mongo_ros::Metadata metadata("x", grasp_vector.pose.position.x, "y", grasp_vector.pose.position.y,
//                                     "z", grasp_vector.pose.position.z);
        double start = episode.when.start.sec + episode.when.start.nsec * pow10(-9);
        double end = episode.when.end.sec + episode.when.end.nsec * pow10(-9);

        MetadataPtr meta = coll_ptr->createMetadata();
        meta->append("uid", (int) episode.uid);
        meta->append("type", episode.type);
        meta->append("source", episode.info.source);
        meta->append("when.start", start);
        meta->append("when.end", end);
        meta->append("relevance.historical.value", episode.relevance.historical.value);
//        meta->append("relevance.historical.last_update", episode.relevance.historical.last_update.);
        return meta;
    }

    bool Manager::insert(const ltm::Episode &episode) {
        _coll->insert(episode, makeMetadata(_coll, episode));
        return true;
    }

    bool Manager::get(int uid, EpisodeWithMetadataPtr& episode_ptr) {
        QueryPtr query = _coll->createQuery();
        query->append("uid", uid);
        try {
            episode_ptr = _coll->findOne(query, false);
        }
        catch (const warehouse_ros::NoMatchingMessageException& exception) {
            episode_ptr.reset();
            return false;
        }
        return true;
    }

    bool Manager::drop_db() {
        _conn.dropDatabase(_db_name);
        setup();
        return true;
    }

    bool Manager::episode_exists(int uid) {
        QueryPtr query = _coll->createQuery();
        query->append("uid", uid);
        try {
            _coll->findOne(query, true);
        }
        catch (const warehouse_ros::NoMatchingMessageException& exception) {
            return false;
        }
        return true;
    }

    std::string Manager::to_short_string(const ltm::Episode &episode) {
        std::stringstream ss;
        ss << "<"
           << "uid:" << episode.uid << ", "
           << "type:" << (int) episode.type << ", "
           << "emotion:" << episode.relevance.emotional.emotion
           << ">";
        return ss.str();
    }

    bool Manager::update_tree_node(int uid, Episode& updated_episode) {
        ROS_DEBUG_STREAM(" - updating node: " << uid);
        EpisodeWithMetadataPtr ep_ptr;

        // get episode
        if (!get(uid, ep_ptr)) {
            throw warehouse_ros::NoMatchingMessageException("episode not found");
        }
        updated_episode = *ep_ptr;

        // is leaf
        if (ep_ptr->type == Episode::LEAF) {
            ROS_DEBUG_STREAM(" - node " << uid << " is a leaf, will not update it.");
            return true;
        }

        bool first_child = true;
        bool result = true;
        std::vector<int32_t>::const_iterator c_it;
        Episode child;
        for (c_it = ep_ptr->children_ids.begin(); c_it != ep_ptr->children_ids.end(); ++c_it) {
            result &= update_tree_node(*c_it, child);

//            int n_child_emotions = (int) child.relevance.emotional.children_emotions.size();
//            if (child.type == Episode::LEAF) {
//
//            }
//            for (int i = 0; i < n_child_emotions; ++i) {
//                int n_emotions = (int) updated_episode.relevance.emotional.children_emotions.size();
//                for (int j = 0; j < ; ++j) {
//
//                }
//
//            }
//            AB.reserve( A.size() + B.size() ); // preallocate memory
//            AB.insert( AB.end(), A.begin(), A.end() );
//            AB.insert( AB.end(), B.begin(), B.end() );


            if (first_child) {
                // WHEN
                updated_episode.when = child.when;

                // Historical Relevance
                updated_episode.relevance.historical = child.relevance.historical;
                first_child = false;


            } else {

                // WHEN
                if (child.when.start < updated_episode.when.start) {
                    updated_episode.when.start = child.when.start;
                }
                if (child.when.end > updated_episode.when.end) {
                    updated_episode.when.end = child.when.end;
                }

                // Historical Relevance
                if (child.relevance.historical.value > updated_episode.relevance.historical.value) {
                    updated_episode.relevance.historical = child.relevance.historical;
                }

            }

        }
        remove_by_uid(uid);
        insert(updated_episode);
        ROS_DEBUG_STREAM(" - node updated: " << uid);
        return result;
    }

    bool Manager::update_tree(int uid) {
        if (!episode_exists(uid)) {
            return false;
        }
        Episode root_ptr;
        return update_tree_node(uid, root_ptr);
    }

}