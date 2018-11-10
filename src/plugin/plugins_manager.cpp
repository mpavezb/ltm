#include <ltm/plugin/plugins_manager.h>
#include <algorithm>

namespace ltm {
    namespace plugin {

        PluginsManager::PluginsManager(DBConnectionPtr ptr, std::string db_name) {
            // empty cache
            registry.clear();

            // Plugin managers
            _location_manager.reset(new LocationManager());
            _emotion_manager.reset(new EmotionManager());
            _streams_manager.reset(new StreamsManager(ptr, db_name));
            _entities_manager.reset(new EntitiesManager(ptr, db_name));
        }

        PluginsManager::~PluginsManager() {
            _location_manager.reset();
            _emotion_manager.reset();
            _streams_manager.reset();
            _entities_manager.reset();
        }

        void PluginsManager::register_episode(uint32_t uid, EpisodeRegister &reg) {
            // register in cache
            std::map<uint32_t, EpisodeRegister>::const_iterator it = registry.find(uid);
            if (it != registry.end()) {
                // episode has already been registered
                ROS_WARN_STREAM("LTM plugin manager: Attempted to register an uid already existing in the cache: " << uid);
                return;
            }

            // register on plugins
            ROS_DEBUG_STREAM("LTM plugin manager: registering episode: " << uid);
            if (reg.gather_location) _location_manager->register_episode(uid);
            if (reg.gather_emotion) _emotion_manager->register_episode(uid);
            if (reg.gather_streams) _streams_manager->register_episode(uid);
            if (reg.gather_entities) _entities_manager->register_episode(uid);
            reg.start = ros::Time::now();
            registry[uid] = reg;
        }

        void PluginsManager::unregister_episode(uint32_t uid) {
            // unregister from cache
            std::map<uint32_t, EpisodeRegister>::const_iterator it = registry.find(uid);
            if (it == registry.end()) {
                // episode has already been unregistered
                return;
            }

            // unregister on plugins
            ROS_DEBUG_STREAM("LTM plugin manager: unregistering episode: " << uid);
            EpisodeRegister reg = registry[uid];
            if (reg.gather_location) _location_manager->unregister_episode(uid);
            if (reg.gather_emotion) _emotion_manager->unregister_episode(uid);
            if (reg.gather_streams) _streams_manager->unregister_episode(uid);
            if (reg.gather_entities) _entities_manager->unregister_episode(uid);
            registry.erase(uid);
        }

        void PluginsManager::collect(uint32_t uid, ltm::Episode &episode) {
            EpisodeRegister reg = registry[uid];
            ROS_DEBUG_STREAM(" - plugin manager: collecting information...");
            ROS_DEBUG_STREAM(" - plugin manager: gather location: " << reg.gather_location);
            ROS_DEBUG_STREAM(" - plugin manager: gather emotion: " << reg.gather_emotion);
            ROS_DEBUG_STREAM(" - plugin manager: gather streams: " << reg.gather_streams);
            ROS_DEBUG_STREAM(" - plugin manager: gather entities: " << reg.gather_entities);
            reg.end = ros::Time::now();
            if (reg.gather_location) _location_manager->collect(uid, episode.where);
            if (reg.gather_emotion)  _emotion_manager->collect(uid, episode.relevance.emotional);
            if (reg.gather_streams) _streams_manager->collect(uid, episode.what, reg.start, reg.end);
            if (reg.gather_entities) _entities_manager->collect(uid, episode.what, reg.start, reg.end);
        }

        void PluginsManager::drop_db() {
            registry.clear();
            ROS_WARN_STREAM("Resetting Location Manager ...");
            _location_manager->reset();

            ROS_WARN_STREAM("Resetting Emotion Manager ...");
            _emotion_manager->reset();

            ROS_WARN_STREAM("Dropping Entity Databases ...");
            _entities_manager->drop_db();

            ROS_WARN_STREAM("Dropping Stream Databases ...");
            _streams_manager->drop_db();
        }

        bool PluginsManager::switch_db(const std::string &db_name) {
            registry.clear();

            ROS_WARN_STREAM("Resetting Location Manager ...");
            _location_manager->reset();

            ROS_WARN_STREAM("Resetting Emotion Manager ...");
            _emotion_manager->reset();

            ROS_WARN_STREAM("Resetting Entity Connections ...");
            _entities_manager->switch_db(db_name);

            ROS_WARN_STREAM("Resseting Stream Connections ...");
            _streams_manager->switch_db(db_name);

        }

        void PluginsManager::append_status(std::stringstream &status) {
            status << "Entity Plugins: \n";
            _entities_manager->append_status(status);
            status << "Stream Plugins: \n";
            _streams_manager->append_status(status);
        }

        void PluginsManager::query_stream(std::string type, const std::string &json, ltm::QueryServer::Response &res) {
            _streams_manager->query(type, json, res);
        }

        void PluginsManager::query_entity(std::string type, const std::string &json, ltm::QueryServer::Response &res, bool trail) {
            _entities_manager->query(type, json, res, trail);
        }

    }
}
