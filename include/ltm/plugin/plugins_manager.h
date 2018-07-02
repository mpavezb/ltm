#ifndef LTM_PLUGINS_MANAGER_H
#define LTM_PLUGINS_MANAGER_H

#include <ltm/plugin/location_manager.h>
#include <ltm/plugin/emotion_manager.h>
#include <ltm/plugin/streams_manager.h>
#include <ltm/plugin/entities_manager.h>

namespace ltm {
    namespace plugin {

        struct EpisodeRegister {
            bool gather_emotion;
            bool gather_location;
            bool gather_streams;
            bool gather_entities;
            ros::Time start;
            ros::Time end;

            EpisodeRegister() {
                gather_emotion = false;
                gather_location = false;
                gather_streams = false;
                gather_entities = false;
                start = ros::Time::now();
            }
        };

        class PluginsManager {
        private:
            // TODO: manejar cache de episodios ya desregistrados! (a pesar de que no existan!)
            // TODO: usar cache para ignorar registro de episodios ya desregistrados (optimizacion)
            typedef boost::shared_ptr<ltm::plugin::LocationManager> LocationManagerPtr;
            typedef boost::shared_ptr<ltm::plugin::EmotionManager> EmotionManagerPtr;
            typedef boost::shared_ptr<ltm::plugin::StreamsManager> StreamsManagerPtr;
            typedef boost::shared_ptr<ltm::plugin::EntitiesManager> EntitiesManagerPtr;

            // cache
            std::map<uint32_t, EpisodeRegister> registry;

            LocationManagerPtr _location_manager;
            EmotionManagerPtr _emotion_manager;
            StreamsManagerPtr _streams_manager;
            EntitiesManagerPtr _entities_manager;

        public:
            PluginsManager(DBConnectionPtr ptr, std::string db_name);
            virtual ~PluginsManager();

            void register_episode(uint32_t uid, EpisodeRegister &reg);
            void unregister_episode(uint32_t uid);
            void collect(uint32_t uid, ltm::Episode &episode);
            void drop_db();
            bool switch_db(const std::string &db_name);
            void append_status(std::stringstream &status);
        };
    }
}

#endif // LTM_PLUGINS_MANAGER_H
