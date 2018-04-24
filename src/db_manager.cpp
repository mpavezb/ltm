#include <ltm/db_manager.h>
#include <ltm/geometry.h>
#include <algorithm>
#include <sstream>

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

    // -----------------------------------------------------------------------------------------------------------------
    // Metadata Builders
    // -----------------------------------------------------------------------------------------------------------------

    void Manager::make_meta_episode(const Episode& node, MetadataPtr meta) {
        meta->append("uid", (int) node.uid);
        meta->append("type", node.type);
        meta->append("parent_id", node.parent_id);

        // TODO: REQUIRES META ARRAY
        // meta->append("children_ids", node.children_ids);
        // meta->append("tags", node.tags);
    }

    void Manager::make_meta_info(const Info& node, MetadataPtr meta) {
        meta->append("info.source", node.source);
    }

    void Manager::make_meta_when(const When& node, MetadataPtr meta) {
        double start = node.start.sec + node.start.nsec * pow10(-9);
        double end = node.end.sec + node.end.nsec * pow10(-9);
        meta->append("when.start", start);
        meta->append("when.end", end);
    }

    void Manager::make_meta_where(const Where& node, MetadataPtr meta) {
        // TODO: how to manage leaf vs node information
        meta->append("where.location", node.location);
        meta->append("where.area", node.area);

        // TODO: REQUIRES META ARRAY
        // meta->append("where.children_locations", node.children_locations);
        // meta->append("where.children_areas", node.children_areas);
    }

    void Manager::make_meta_what(const What& node, MetadataPtr meta) {
        // TODO: REQUIRES META ARRAY
        // meta->append("what.features", node.features);
    }

    void Manager::make_meta_relevance(const Relevance& node, MetadataPtr meta) {
        make_meta_relevance_emotional(node.emotional, meta);
        make_meta_relevance_historical(node.historical, meta);
    }

    void Manager::make_meta_relevance_historical(const HistoricalRelevance& node, MetadataPtr meta) {
        meta->append("relevance.historical.value", node.value);

        char buff[16];
        snprintf(buff, sizeof(buff), "%04u/%02u/%02u", node.last_update.year, node.last_update.month, node.last_update.day);
        std::string last_update = buff;
        snprintf(buff, sizeof(buff), "%04u/%02u/%02u", node.next_update.year, node.next_update.month, node.next_update.day);
        std::string next_update = buff;
        meta->append("relevance.historical.last_update", last_update);
        meta->append("relevance.historical.next_update", next_update);
    }

    void Manager::make_meta_relevance_emotional(const EmotionalRelevance& node, MetadataPtr meta) {
        meta->append("relevance.emotional.emotion", node.emotion);
        meta->append("relevance.emotional.value", node.value);

        // TODO: REQUIRES META ARRAY
        // meta->append("relevance.emotional.children_emotions", node.children_emotions);
        // meta->append("relevance.emotional.children_values", node.children_values);
    }

    MetadataPtr Manager::make_metadata(const Episode &episode) {
        MetadataPtr meta = _coll->createMetadata();
        make_meta_episode(episode, meta);
        make_meta_info(episode.info, meta);
        make_meta_when(episode.when, meta);
        make_meta_where(episode.where, meta);
        make_meta_what(episode.what, meta);
        make_meta_relevance(episode.relevance, meta);
        return meta;
    }


    // -----------------------------------------------------------------------------------------------------------------
    // Update Queries
    // -----------------------------------------------------------------------------------------------------------------

    std::string Manager::vector_to_str(const std::vector<std::string>& array) {
        std::vector<std::string>::const_iterator it;
        std::stringstream ss;
        ss << "[";
        for (it = array.begin(); it != array.end(); ++it) {
            ss << *it << ", ";
        }
        ss.seekp(-2, ss.cur);
        ss << "]";
        return ss.str();
    }

    void Manager::vector_merge(std::vector<std::string>& result, const std::vector<std::string>& source) {
        std::vector<std::string>::const_iterator it;
        for (it = source.begin(); it != source.end(); ++it) {

            if (std::find(result.begin(), result.end(), *it) == result.end()) {
                result.push_back(*it);
            }

        }
        std::sort(result.begin(), result.end());
    }

    bool Manager::update_tree_tags(Episode& node, const Episode& child) {
        vector_merge(node.tags, child.tags);
        ROS_DEBUG_STREAM(" ---> tags: " << vector_to_str(node.tags));
        return true;
    }

    bool Manager::update_tree_info(Info& node, const Info& child, bool first, bool is_leaf) {
        if (first) {
            node = child;
            return true;
        }

        int n_usages = node.n_usages;
        if (node.creation_date < child.creation_date) {
            node = child;
        }
        // TODO: register this: n_usages parent = SUM children
        node.n_usages += n_usages;
        return true;
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

    bool Manager::update_tree_where_last(Where& node, std::vector<geometry_msgs::Point> &positions) {
        ROS_DEBUG_STREAM(" ---> where (pre hull): " << point_vector_to_str(positions));
        // returns ordered list
        convex_hull_2d(positions, node.children_hull);

        // this requires an ordered list
        node.position = polygon_centroid_2d(node.children_hull);
        ROS_DEBUG_STREAM(" ---> where.children_hull: " << point_vector_to_str(node.children_hull));
        ROS_DEBUG_STREAM(" ---> where.position: (" << node.position.x << ", " << node.position.y << ")");
        return true;
    }

    bool Manager::update_tree_where(Where& node, const Where& child, bool first, bool is_leaf, int node_uid, int child_uid, std::vector<geometry_msgs::Point> &positions) {

        // node is never a leaf!
        node.location = "";
        node.area = "";

        if (is_leaf) {
            std::vector<std::string> child_location;
            child_location.push_back(child.location);
            vector_merge(node.children_locations, child_location);

            std::vector<std::string> child_area;
            child_location.push_back(child.area);
            vector_merge(node.children_locations, child_area);

            // just a point
            positions.push_back(child.position);
        } else {
            vector_merge(node.children_locations, child.children_locations);
            vector_merge(node.children_locations, child.children_areas);

            // join children hull + other points
            positions.reserve(positions.size() + child.children_hull.size());
            positions.insert(positions.end(), child.children_hull.begin(), child.children_hull.end());
        }

        // TODO: WRITE limitation SOMEWHERE: require same frame_id and map_name on a tree
        // frame_id and map_name must be the same for all children
        bool result = true;
        if (first) {
            node.frame_id = child.frame_id;
            node.map_name = child.map_name;
        } else {
            if (node.frame_id != child.frame_id) {
                ROS_WARN_STREAM(
                        "UPDATE: parent ("
                        << node_uid << ") where.frame_id {" << node.frame_id
                        << "} does not match child's (" << child_uid << ") {" << child.frame_id << "}.");
                result = false;
            }
            if (node.map_name != child.map_name) {
                ROS_WARN_STREAM(
                        "UPDATE: parent ("
                        << node_uid << ") where.map_name {" << node.map_name
                        << "} does not match child's (" << child_uid << ") {" << child.map_name << "}.");
                result = false;
            }
        }
        return result;
    }

    bool Manager::update_tree_what(What& node, const What& child, bool first, bool is_leaf) {
        // TODO
        return true;
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

        return true;
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
            ROS_DEBUG_STREAM(" ---> node " << uid << " is a leaf, will not update it.");
            return true;
        }

        bool first_child = true;
        bool is_leaf;
        bool result = true;
        std::vector<geometry_msgs::Point> positions;
        std::vector<int32_t>::const_iterator c_it;
        Episode child;
        for (c_it = ep_ptr->children_ids.begin(); c_it != ep_ptr->children_ids.end(); ++c_it) {

            result = result && update_tree_node(*c_it, child);
            is_leaf = (child.type == Episode::LEAF);

            // update components
            result = result && update_tree_tags(updated_episode, child);
            result = result && update_tree_info(updated_episode.info, child.info, first_child, is_leaf);
            result = result && update_tree_when(updated_episode.when, child.when, first_child);
            result = result && update_tree_where(updated_episode.where, child.where, first_child, is_leaf, uid, child.uid, positions);
            result = result && update_tree_what(updated_episode.what, child.what, first_child, is_leaf);
            result = result && update_tree_relevance(updated_episode.relevance, child.relevance, first_child, is_leaf);
            first_child = false;
        }
        result = result && update_tree_where_last(updated_episode.where, positions);

        // TODO: update instead of remove/insert
        result = result && remove(uid);
        result = result && insert(updated_episode);
        ROS_DEBUG_STREAM(" -> node updated: " << uid);
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
        _coll->insert(episode, make_metadata(episode));
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