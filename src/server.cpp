#include <ltm/server.h>
#include <boost/scoped_ptr.hpp>

using warehouse_ros::Metadata;


namespace ltm {

    inline ltm::Episode makeEpisode(const u_int32_t uid, const u_int8_t type, const u_int32_t parent_id)
    {
        ltm::Episode episode;
        episode.uid = uid;
        episode.type = type;
        episode.what.parent_id = parent_id;
        episode.info.source = "test";
        return episode;
    }

    Metadata::Ptr makeMetadata(const EpisodeCollection& coll, const ltm::Episode& episode)
    {
        Metadata::Ptr meta = coll.createMetadata();
        meta->append("uid", (int) episode.uid);
        meta->append("type", episode.type);
        meta->append("source", episode.info.source);
        return meta;
    }

    Server::Server() {
        ros::NodeHandle priv("~");

        // setup
        _conn.setParams("localhost", 27017, 60.0);
        _conn.connect();

        // clear existing data
        _conn.dropDatabase("ltm_db");

        // open the collection


        // indexes
        // collection.

        // Announce services
        _status_service = priv.advertiseService("status", &Server::status, this);
        _append_dummies_service = priv.advertiseService("append_dummies", &Server::appendDummies, this);

        ROS_INFO("LTM server is up and running.");
    }

    Server::~Server() {}

    bool Server::status(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        EpisodeCollection _coll = _conn.openCollection<ltm::Episode>("ltm_db", "episodes");
        ROS_INFO_STREAM(std::endl
                << std::endl << "LTM server status"
                << std::endl << "================="
                << std::endl << "Number of episodes: " << _coll.count()
                << std::endl
        );
        return true;
    }

    bool Server::appendDummies(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
        EpisodeCollection _coll = _conn.openCollection<ltm::Episode>("ltm_db", "episodes");

        // create dummy episodes
        const ltm::Episode ep1 = makeEpisode(1, ltm::Episode::LEAF, 3);
        const ltm::Episode ep2 = makeEpisode(2, ltm::Episode::LEAF, 3);
        const ltm::Episode ep3 = makeEpisode(3, ltm::Episode::EPISODE, 3);
        const ltm::Episode ep4 = makeEpisode(4, ltm::Episode::CONTEXT, 0);
        const ltm::Episode ep5 = makeEpisode(5, ltm::Episode::LEAF, 7);
        const ltm::Episode ep6 = makeEpisode(6, ltm::Episode::LEAF, 7);
        const ltm::Episode ep7 = makeEpisode(7, ltm::Episode::EPISODE, 4);

        // insert data
        _coll.insert(ep1, makeMetadata(_coll, ep1));
        _coll.insert(ep2, makeMetadata(_coll, ep2));
        _coll.insert(ep3, makeMetadata(_coll, ep3));
        _coll.insert(ep4, makeMetadata(_coll, ep4));
        _coll.insert(ep5, makeMetadata(_coll, ep5));
        _coll.insert(ep6, makeMetadata(_coll, ep6));
        _coll.insert(ep7, makeMetadata(_coll, ep7));

        res.success = (u_int8_t) true;
        res.message = "Appended 7 msgs";
        return true;
    }
}


int main(int argc, char **argv) {
    // init node
    ros::init(argc, argv, "ltm_server");
    boost::scoped_ptr<ltm::Server> server(new ltm::Server());

    // run
    ros::spin();

    // close
    printf("\nClosing LTM server... \n\n");
    return 0;
}

