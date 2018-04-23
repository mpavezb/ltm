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


    // =================================================================================================================
    // Private API
    // =================================================================================================================

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

    bool Manager::update_tree_tags(Episode& node, const Episode& child, bool first, bool is_leaf) {

    }

    bool Manager::update_tree_info(Info& node, const Info& child, bool first, bool is_leaf) {

    }

    bool Manager::update_tree_when(When& node, const When& child, bool first) {
        if (first) {
            node = child;
        } else {
            if (child.start < node.start) {
                node.start = child.start;
            }
            if (child.end > node.end) {
                node.end = child.end;
            }
        }
        return true;
    }

    bool Manager::update_tree_where(Where& node, const Where& child, bool first, bool is_leaf) {

    }

    bool Manager::update_tree_what(What& node, const What& child, bool first, bool is_leaf) {

    }

    bool Manager::update_tree_relevance(Relevance& node, const Relevance& child, bool first, bool is_leaf) {
        bool result = true;
        result = result && update_tree_relevance_historical(node.historical, child.historical, first);
        result = result && update_tree_relevance_emotional(node.emotional, child.emotional, first, is_leaf);
        return result;
    }

    bool Manager::update_tree_relevance_historical(HistoricalRelevance& node, const HistoricalRelevance& child, bool first) {
        if (first) {
            node = child;
        } else {
            if (child.value > node.value) {
                node = child;
            }
        }
        return true;
    }

    bool Manager::update_tree_relevance_emotional(EmotionalRelevance& node, const EmotionalRelevance& child, bool first, bool is_leaf) {

        //            int n_child_emotions = (int) child.relevance.emotional.children_emotions.size();

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
        bool is_leaf;
        bool result = true;
        std::vector<int32_t>::const_iterator c_it;
        Episode child;
        for (c_it = ep_ptr->children_ids.begin(); c_it != ep_ptr->children_ids.end(); ++c_it) {

            result = result && update_tree_node(*c_it, child);
            is_leaf = (child.type == Episode::LEAF);

            // update components
            update_tree_tags(updated_episode, child, first_child, is_leaf);
            update_tree_info(updated_episode.info, child.info, first_child, is_leaf);
            update_tree_when(updated_episode.when, child.when, first_child, is_leaf);
            update_tree_where(updated_episode.where, child.where, first_child, is_leaf);
            update_tree_what(updated_episode.what, child.what, first_child, is_leaf);
            update_tree_relevance(updated_episode.relevance, child.relevance, first_child, is_leaf);
            first_child = false;
        }
        remove(uid);
        insert(updated_episode);
        ROS_DEBUG_STREAM(" - node updated: " << uid);
        return result;
    }

    // =================================================================================================================
    // Public API
    // =================================================================================================================

    std::string Manager::to_short_string(const ltm::Episode &episode) {
        std::stringstream ss;
        ss << "<"
           << "uid:" << episode.uid << ", "
           << "type:" << (int) episode.type << ", "
           << "emotion:" << episode.relevance.emotional.emotion
           << ">";
        return ss.str();
    }

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

    // -----------------------------------------------------------------------------------------------------------------
    // CRUD methods
    // -----------------------------------------------------------------------------------------------------------------

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

    bool Manager::update(const ltm::Episode &episode) {
        ROS_WARN("UPDATE: Method not implemented");
        return false;
    }

    bool Manager::remove(int uid) {
        QueryPtr query = _coll->createQuery();
        query->append("uid", uid);
        _coll->removeMessages(query);
        return true;
    }


    // -----------------------------------------------------------------------------------------------------------------
    // Other Queries
    // -----------------------------------------------------------------------------------------------------------------

    int Manager::count() {
        return _coll->count();
    }

    bool Manager::has(int uid) {
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

    bool Manager::update_tree(int uid) {
        if (!has(uid)) {
            return false;
        }
        Episode root_ptr;
        return update_tree_node(uid, root_ptr);
    }

    bool Manager::drop_db() {
        _conn.dropDatabase(_db_name);
        setup();
        return true;
    }

}