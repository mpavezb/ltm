#include <ltm/parameter_server_wrapper.h>
#include <ltm/server.h>
#include <boost/scoped_ptr.hpp>

using warehouse_ros::Metadata;


namespace ltm {

    Server::Server() {
        ros::NodeHandle priv("~");

        ParameterServerWrapper psw;
        psw.getParameter("db", _db_name, "ltm_db");
        psw.getParameter("collection", _db_collection_name, "episodes");
        psw.getParameter("host", _db_host, "localhost");
        psw.getParameter("port", _db_port, 27017);
        psw.getParameter("timeout", _db_timeout, 60.0);

        // DB manager
        _db.reset(new Manager(_db_name, _db_collection_name, _db_host, (uint)_db_port, _db_timeout));
        _db->setup();

        // Announce services
        _get_episode_service = priv.advertiseService("get_episode", &Server::get_episode_service, this);
        _update_tree_service = priv.advertiseService("update_tree", &Server::update_tree_service, this);
        _add_episode_service = priv.advertiseService("add_episode", &Server::add_episode_service, this);
        _status_service = priv.advertiseService("status", &Server::status_service, this);
        _drop_db_service = priv.advertiseService("drop_db", &Server::drop_db_service, this);

        ROS_INFO("LTM server is up and running.");
        show_status();
    }

    Server::~Server() {}

    void Server::show_status() {
        ROS_INFO_STREAM("DB has " << _db->count() << " entries.");
    }

    // ==========================================================
    // ROS Services
    // ==========================================================

    bool Server::get_episode_service(ltm::GetEpisode::Request &req, ltm::GetEpisode::Response &res) {
        ROS_INFO_STREAM("GET: Retrieving episode with uid: " << req.uid);
        EpisodeWithMetadataPtr ep_ptr;
        if (!_db->get(req.uid, ep_ptr)) {
            ROS_ERROR_STREAM("GET: Episode with uid '" << req.uid << "' not found.");
            res.succeeded = (uint8_t) false;
            return true;
        }
        res.episode = *ep_ptr;
        res.succeeded = (uint8_t) true;
        return true;
    }

    bool Server::update_tree_service(ltm::UpdateTree::Request &req, ltm::UpdateTree::Response &res) {
        ROS_INFO_STREAM("Updating episode structure for uid: " << req.uid);
        // reportar problemas!
        // TODO: missing child
        // TODO: missing root
        if (!_db->update_tree(req.uid)) {
            ROS_ERROR_STREAM("A problem occurred while trying to update the tree for uid: " << req.uid);
            res.succeeded = (uint8_t) false;
        }
        res.succeeded = (uint8_t) true;
        return true;
    }

    bool Server::add_episode_service(ltm::AddEpisode::Request &req, ltm::AddEpisode::Response &res) {
        bool replace = false;
        if (_db->episode_exists(req.episode.uid)) {
            if (!req.replace) {
                ROS_ERROR_STREAM("ADD: Episode with uid '" << req.episode.uid << "' already exists.");
                res.succeeded = (uint8_t) false;
                return true;
            }
            replace = true;
            _db->remove_by_uid(req.episode.uid);
        }
        _db->insert(req.episode);
        ROS_INFO_STREAM_COND(replace, "ADD: Replacing episode '" << req.episode.uid << "'. (" << _db->count() << " entries)");
        ROS_INFO_STREAM_COND(!replace, "ADD: New episode '" << req.episode.uid << "'. (" << _db->count() << " entries)");
        res.succeeded = (uint8_t) true;
        return true;
    }

    bool Server::status_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        show_status();
        return true;
    }

    bool Server::drop_db_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        // TODO: this requires a synchronization mechanism
        // TODO: the connection should be closed first?
        ROS_INFO("DELETE: Dropping DB");
        _db->drop_db();
        show_status();
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

