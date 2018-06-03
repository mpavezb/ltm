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
            reg.end = ros::Time::now();
            if (reg.gather_location) _location_manager->collect(uid, episode.where);
            if (reg.gather_emotion)  _emotion_manager->collect(uid, episode.relevance.emotional);
            if (reg.gather_streams) _streams_manager->collect(uid, episode.what, reg.start, reg.end);
            if (reg.gather_entities) _entities_manager->collect(uid, episode.what, reg.start, reg.end);
            unregister_episode(uid);
        }
    }
}
