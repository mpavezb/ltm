#include <ltm/util/parameter_server_wrapper.h>
#include <ltm/server.h>
#include <boost/scoped_ptr.hpp>

using ltm_db::Metadata;


namespace ltm {

    Server::Server() {
        ros::NodeHandle priv("~");
        _log_prefix = "[LTM]: ";

        ltm::util::ParameterServerWrapper psw;
        psw.getParameter("db", _db_name, "ltm_db");
        psw.getParameter("collection", _db_collection_name, "episodes");
        psw.getParameter("host", _db_host, "localhost");
        psw.getParameter("port", _db_port, 27017);
        psw.getParameter("timeout", _db_timeout, 60.0);

        // DB manager
        _db.reset(new ltm::db::EpisodeCollectionManager(_db_name, _db_collection_name, _db_host, (uint)_db_port, _db_timeout));
        _db->setup();

        // Plugins manager
        _pl.reset(new ltm::plugin::PluginsManager(_db->_conn, _db_name));

        // Announce services
        _add_episode_service = priv.advertiseService("episode/add", &Server::add_episode_service, this);
        _get_episodes_service = priv.advertiseService("episode/get", &Server::get_episodes_service, this);
        _register_episode_service = priv.advertiseService("episode/register", &Server::register_episode_service, this);
        _update_tree_service = priv.advertiseService("episode/update_tree", &Server::update_tree_service, this);
        _status_service = priv.advertiseService("db/status", &Server::status_service, this);
        _drop_db_service = priv.advertiseService("db/drop", &Server::drop_db_service, this);
        _switch_db_service = priv.advertiseService("db/switch", &Server::switch_db_service, this);
        _query_server_service = priv.advertiseService("db/query", &Server::query_server_service, this);

        ROS_INFO_STREAM(_log_prefix << "Server is up and running.");
        show_status();
    }

    Server::~Server() {}

    void Server::show_status() {
        std::stringstream status;
        status << "Database parameters: " << std::endl;
        status << " - name: " << _db_name << std::endl;
        status << " - host: " << _db_host << std::endl;
        status << " - port: " << _db_port << std::endl;
        status << "Episodes: " << _db->count() << " entries in collection '" << _db_collection_name << "'" << std::endl;
        _pl->append_status(status);
        ROS_INFO_STREAM(_log_prefix << "DB Status:\n" << status.str());
    }

    // ==========================================================
    // ROS Services
    // ==========================================================

    bool Server::get_episodes_service(ltm::GetEpisodes::Request &req, ltm::GetEpisodes::Response &res) {
//        ROS_INFO_STREAM(_log_prefix << "GET: Retrieving episode with uid: " << req.uids);
//        EpisodeWithMetadataPtr ep_ptr;
//        if (!_db->get(req.uid, ep_ptr)) {
//            ROS_ERROR_STREAM(_log_prefix << "GET: Episode with uid '" << req.uid << "' not found.");
//            res.succeeded = (uint8_t) false;
//            return true;
//        }
//        res.episodes = *ep_ptr;
//        res.succeeded = (uint8_t) true;
//        return true;
    }

    bool Server::query_server_service(ltm::QueryServer::Request &req, ltm::QueryServer::Response &res) {
        if (req.target == "episode") {
            _db->query(req.json, res);
            return true;
        } else if (req.target == "entity") {
            _pl->query_entity(req.semantic_type, req.json, res);
            return true;
        } else if (req.target == "stream") {
            _pl->query_stream(req.semantic_type, req.json, res);
            return true;
        }
        ROS_WARN_STREAM("Invalid 'target' field for 'query' service. Got: '" << req.target << "'");
        return false;
    }

    bool Server::register_episode_service(ltm::RegisterEpisode::Request &req, ltm::RegisterEpisode::Response &res) {
        int value;
        if (req.generate_uid) {
            value = _db->reserve_uid();
            ROS_DEBUG_STREAM(_log_prefix << "[REGISTER] Reserving random uid: " << value);
        } else {
            value = req.uid;
            ROS_DEBUG_STREAM(_log_prefix << "[REGISTER] Reserving fixed uid: " << value);
            if (_db->is_reserved(value)) {
                if (req.replace) {
                    ROS_INFO_STREAM(_log_prefix << "[REGISTER] Removing episode (" << value << ") for replacement.");
                    _db->remove(value);
                    _pl->unregister_episode((uint32_t) value);
                } else {
                    ROS_WARN_STREAM(_log_prefix << "[REGISTER] Attempted to register a fixed uid (" << value << "), but it is already registered.");
                    return false;
                }
            }
        }
        res.uid = (uint32_t) value;

        // Only LEAVES get registered on plugins
        if (value >= 0 && req.is_leaf) {
            ltm::plugin::EpisodeRegister reg;
            reg.gather_emotion = req.gather_emotion;
            reg.gather_location = req.gather_location;
            reg.gather_streams = req.gather_streams;
            reg.gather_entities = req.gather_entities;
            _pl->register_episode(res.uid, reg);
        }
        return (value >= 0);
    }

    bool Server::update_tree_service(ltm::UpdateTree::Request &req, ltm::UpdateTree::Response &res) {
        ROS_INFO_STREAM(_log_prefix << "Updating episode structure for uid: " << req.uid);
        // reportar problemas!
        // TODO: missing child
        // TODO: missing root
        if (!_db->update_tree(req.uid)) {
            ROS_ERROR_STREAM(_log_prefix << "A problem occurred while trying to update the tree for uid: " << req.uid);
            res.succeeded = (uint8_t) false;
        }
        res.succeeded = (uint8_t) true;
        return true;
    }

    bool Server::add_episode_service(ltm::AddEpisode::Request &req, ltm::AddEpisode::Response &res) {
        bool replace = false;
        if (req.episode.type == ltm::Episode::LEAF) {
            // only collect information for LEAFs
            _pl->collect(req.episode.uid, req.episode);
        } else if (req.episode.type == ltm::Episode::EPISODE) {
            // update node based on its children
            // THIS REQUIRES THE CHILDREN_IDS FIELD TO BE SET UP.
            // TODO: rework design to not require this field. It can be built automatically while adding children.
            _db->update_from_children(req.episode);
        } else {
            ROS_ERROR_STREAM("Unsupported episode type: " << (uint32_t) req.episode.type);
            return false;
        }

        // insert episode
        if (_db->has(req.episode.uid)) {
            if (!req.replace) {
                ROS_ERROR_STREAM(_log_prefix << "ADD: Episode with uid '" << req.episode.uid << "' already exists.");
                res.succeeded = (uint8_t) false;
                // FIXME: no retornar. desuscribir
                return true;
            }
            replace = true;
            _db->remove(req.episode.uid);
        }
        _db->insert(req.episode);

        // TODO: update branch up to the root
        // impl

        // finish
        ROS_INFO_STREAM_COND(replace, _log_prefix << "ADD: Replacing episode '" << req.episode.uid << "'. (" << _db->count() << " entries)");
        ROS_INFO_STREAM_COND(!replace, _log_prefix << "ADD: New episode '" << req.episode.uid << "'. (" << _db->count() << " entries)");
        res.succeeded = (uint8_t) true;
        return true;
    }

    bool Server::status_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        show_status();
        return true;
    }

    bool Server::drop_db_service(ltm::DropDB::Request &req, ltm::DropDB::Response &res) {
        // TODO: this requires a synchronization mechanism
        if (!req.i_understand_this_is_a_dangerous_operation ||
            req.html_is_a_real_programming_language) {
            ROS_WARN_STREAM(_log_prefix << "Attempted to drop the database with unmet requirements!. Please read "
                                           << "the ltm/DropDB.srv description. Bye!.");
            return false;
        }

        ROS_WARN_STREAM(_log_prefix << "DELETE: Deleting all entries from collection '" << _db_collection_name << "'");
        _pl->drop_db();
        _db->drop_db();
        show_status();
        return true;
    }

    bool Server::switch_db_service(ltm::SwitchDB::Request &req, ltm::SwitchDB::Response &res) {
        ROS_WARN_STREAM(_log_prefix << "Switching LTM database from '" << _db_name << "' to '" << req.db_name << "'");
        _db_name = req.db_name;
        _pl->switch_db(req.db_name);
        _db->switch_db(req.db_name);
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

