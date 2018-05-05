#include <ltm/db_manager.h>
#include <ltm/geometry.h>
#include <algorithm>
#include <sstream>

// get random uid
#include <time.h>
#include <limits>
#include <stdlib.h>

namespace ltm {

    Manager::Manager(const std::string &name, const std::string &collection, const std::string &host, uint port,
                     float timeout) {
        _db_name = name;
        _db_collection_name = collection;
        _db_host = host;
        _db_port = port;
        _db_timeout = timeout;

        // reserved uids are cleared on startup
        _reserved_uids.clear();
        _db_uids.clear();
        std::srand(time(NULL));
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
        // meta->append("children_tags", node.children_tags);
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
    // Update Tree Queries
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

    void Manager::update_tree_tags_init(Episode &node) {
        // do not clear own tags!.
        node.children_tags.clear();
    }

    bool Manager::update_tree_tags(Episode& node, const Episode& child) {
        vector_merge(node.children_tags, child.tags);
        vector_merge(node.children_tags, child.children_tags);
        ROS_DEBUG_STREAM(" ---> tags: " << vector_to_str(node.tags));
        ROS_DEBUG_STREAM(" ---> child tags: " << vector_to_str(node.children_tags));
        return true;
    }

    void Manager::update_tree_info_init(Info &node) {
        node.n_usages = 0;
        node.creation_date = ros::Time(0, 0);
    }

    bool Manager::update_tree_info(Info& node, const Info& child) {
        // TODO: register this: n_usages parent = SUM children
        int n_usages = node.n_usages;
        if (node.creation_date < child.creation_date) {
            node = child;
        }
        node.n_usages += n_usages;
        return true;
    }

    void Manager::update_tree_when_init(When &node) {
        node.start = ros::Time::now();
        node.end = ros::Time(0, 0);
    }

    bool Manager::update_tree_when(When& node, const When& child) {
        if (child.start < node.start) {
            node.start = child.start;
        }
        if (child.end > node.end) {
            node.end = child.end;
        }
        return true;
    }

    void Manager::update_tree_where_init(Where &node) {
        // node is never a leaf!
        node.location = "";
        node.area = "";
        node.frame_id = "";
        node.map_name = "";
        node.children_locations.clear();
        node.children_areas.clear();
        node.children_hull.clear();
        node.position = geometry_msgs::Point();
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

    bool Manager::update_tree_where(Where& node, const Where& child, bool is_leaf, int node_uid, int child_uid, std::vector<geometry_msgs::Point> &positions) {
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
        if (node.map_name == "" && node.frame_id == "") {
            // first time
            node.frame_id = child.frame_id;
            node.map_name = child.map_name;
        } else {
            // next children checks
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

    void Manager::update_tree_what_init(What &node) {
        // TODO
    }

    bool Manager::update_tree_what(What& node, const What& child, bool is_leaf) {
        // TODO
        return true;
    }

    void Manager::update_tree_relevance_init(Relevance &node) {
        update_tree_relevance_historical_init(node.historical);
        update_tree_relevance_emotional_init(node.emotional);
    }

    bool Manager::update_tree_relevance(Relevance& node, const Relevance& child, bool is_leaf) {
        bool result = true;
        result = result && update_tree_relevance_historical(node.historical, child.historical);
        result = result && update_tree_relevance_emotional(node.emotional, child.emotional, is_leaf);
        return result;
    }

    void Manager::update_tree_relevance_historical_init(HistoricalRelevance &node) {
        node.value = 0;
    }

    bool Manager::update_tree_relevance_historical(HistoricalRelevance& node, const HistoricalRelevance& child) {
        if (child.value > node.value) {
            node = child;
        }
        return true;
    }

    void Manager::update_tree_relevance_emotional_init(EmotionalRelevance &node) {
        // init with ordered emotions: 0 to 7
        static const int arr[] = {0, 1, 2, 3, 4, 5, 6, 7};
        std::vector<int32_t> tmp (arr, arr + sizeof(arr) / sizeof(arr[0]));
        node.children_emotions.clear();
        node.children_emotions = tmp;

        static const float arr2[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<float> tmp2 (arr, arr + sizeof(arr2) / sizeof(arr2[0]));
        node.children_values.clear();
        node.children_values = tmp2;

        // init global values with zeros
        node.emotion = EmotionalRelevance::JOY;
        node.value = 0.0;

        // TODO: WRITE THIS SOMEWHERE parent does not uses this information
        node.software = "";
        node.software_version = "";
        node.registered_emotions.clear();
        node.registered_values.clear();
    }

    bool Manager::update_tree_relevance_emotional(EmotionalRelevance& node, const EmotionalRelevance& child, bool is_leaf) {
        // TODO: WRITE THIS: children_emotions/values are keep ordered by emotion number
        // TODO: WRITE THIS keep max values on arrays and single field.
        ROS_DEBUG_STREAM(" --> emo: node (pre) (" << node.emotion << ", " << node.value << ")");
        ROS_DEBUG_STREAM(" -->      child      (" << child.emotion << ", " << child.value << ")");
        if (is_leaf) {
            // update just one emotion
            if (child.value > node.children_values[child.emotion]) {
                node.children_values[child.emotion] = child.value;
            }
        } else {
            // update all emotions
            std::vector<int32_t >::const_iterator e_it = child.children_emotions.begin();
            std::vector<float>::const_iterator f_it = child.children_values.begin();
            for (; e_it != child.children_emotions.end() && f_it != child.children_values.end(); ++e_it, ++f_it) {
                if (*f_it > node.children_values[*e_it]) {
                    // keep max on list
                    node.children_values[*e_it] = *f_it;
                }
            }

        }

        // update max emotion
        if (child.value > node.value) {
            node.emotion = child.emotion;
            node.value = child.value;
        }
        ROS_DEBUG_STREAM(" -->      node (post)(" << node.emotion << ", " << node.value << ")");
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

        // init fields
        update_tree_tags_init(updated_episode);
        update_tree_info_init(updated_episode.info);
        update_tree_when_init(updated_episode.when);
        update_tree_where_init(updated_episode.where);
        update_tree_what_init(updated_episode.what);
        update_tree_relevance_init(updated_episode.relevance);

        // process children
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
            result = result && update_tree_info(updated_episode.info, child.info);
            result = result && update_tree_when(updated_episode.when, child.when);
            result = result && update_tree_where(updated_episode.where, child.where, is_leaf, uid, child.uid, positions);
            result = result && update_tree_what(updated_episode.what, child.what, is_leaf);
            result = result && update_tree_relevance(updated_episode.relevance, child.relevance, is_leaf);
        }
        // finish fields
        result = result && update_tree_where_last(updated_episode.where, positions);

        // save updated episode
        // TODO: update instead of remove/insert
        result = result && remove(uid);
        result = result && insert(updated_episode);
        ROS_DEBUG_STREAM(" -> node updated: " << uid);
        ROS_ERROR_STREAM_COND(!result,"UPDATE TREE: An error occurred while updating episode (" << uid << ")");
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

    int Manager::reserve_uid() {
        // TODO: find better way, maybe select the max_uid+1
        // Returns a pseudo-random integral number in the range between 0 and RAND_MAX.
        uint32_t max_int32 = std::numeric_limits<uint32_t>::max();
        int max_int = std::numeric_limits<int>::max();
        int upper_bound = max_int32 > max_int ? max_int : max_int32;

        int db_size = count();
        int reserved_ids = (int)_reserved_uids.size();
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