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

        MetadataPtr meta = coll_ptr->createMetadata();
        meta->append("uid", (int) episode.uid);
        meta->append("type", episode.type);
        meta->append("source", episode.info.source);
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

}