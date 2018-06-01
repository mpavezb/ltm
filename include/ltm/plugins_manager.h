#ifndef LTM_PLUGINS_MANAGER_H
#define LTM_PLUGINS_MANAGER_H

#include <pluginlib/class_loader.h>
#include <ltm/plugin/plugins_base.h>

typedef boost::shared_ptr<ltm::plugin::EmotionBase> EmotionPluginPtr;
typedef boost::shared_ptr<ltm::plugin::LocationBase> LocationPluginPtr;
typedef boost::shared_ptr<ltm::plugin::StreamBase> StreamPluginPtr;
typedef boost::shared_ptr<ltm::plugin::EntityBase> EntityPluginPtr;

namespace ltm {

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

        // class loaders
        pluginlib::ClassLoader<ltm::plugin::EmotionBase> *_emotion_loader;
        pluginlib::ClassLoader<ltm::plugin::LocationBase> *_location_loader;
        pluginlib::ClassLoader<ltm::plugin::StreamBase> *_stream_loader;
        pluginlib::ClassLoader<ltm::plugin::EntityBase> *_entity_loader;

        // plugins
        EmotionPluginPtr _emotion_pl;
        LocationPluginPtr _location_pl;
        std::vector<StreamPluginPtr> _stream_pls;
        std::vector<EntityPluginPtr> _entity_pls;

        // params
        bool _use_emotion_pl;
        bool _use_location_pl;
        bool _use_stream_pls;
        bool _use_entity_pls;

        // data
        std::map<uint32_t, ltm::EpisodeRegister> registry;
        DBConnectionPtr _conn;
        std::string _db_name;

        void load_emotion_plugin(std::string plugin_class);
        void load_location_plugin(std::string plugin_class);
        void load_stream_plugins(const std::vector<std::string> &plugin_classes);
        void load_entity_plugins(const std::vector<std::string> &plugin_classes);

        void collect_emotion(uint32_t uid, ltm::EmotionalRelevance &msg);
        void collect_location(uint32_t uid, ltm::Where &msg);
        void collect_streams(uint32_t uid, ltm::What &msg, ros::Time start, ros::Time end);
        void collect_entities(uint32_t uid, ltm::What &msg, ros::Time start, ros::Time end);

    public:
        PluginsManager();
        void setup(DBConnectionPtr ptr, std::string db_name);
        void register_episode(uint32_t uid, ltm::EpisodeRegister& reg);
        void unregister_episode(uint32_t uid);
        void collect(uint32_t uid, ltm::Episode& episode);

        virtual ~PluginsManager();
    };
}

#endif // LTM_PLUGINS_MANAGER_H
