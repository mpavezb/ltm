#ifndef LTM_PLUGINS_MANAGER_H
#define LTM_PLUGINS_MANAGER_H

#include <pluginlib/class_loader.h>
#include <ltm/plugins_base.h>
#include <ltm/Episode.h>

typedef boost::shared_ptr<ltm::plugin::EmotionBase> EmotionPluginPtr;
typedef boost::shared_ptr<ltm::plugin::LocationBase> LocationPluginPtr;
typedef boost::shared_ptr<ltm::plugin::StreamBase> StreamPluginPtr;
typedef boost::shared_ptr<ltm::plugin::EntityBase> EntityPluginPtr;

namespace ltm {

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
        std::vector<uint32_t> registry;

        void load_emotion_plugin(std::string plugin_class);
        void load_location_plugin(std::string plugin_class);
        void load_stream_plugins(const std::vector<std::string> &plugin_classes);
        void load_entity_plugins(const std::vector<std::string> &plugin_classes);

    public:
        PluginsManager();
        void setup();
        void register_episode(uint32_t uid);
        void unregister_episode(uint32_t uid);
        void collect_emotion(uint32_t uid, ltm::EmotionalRelevance &msg);
        void collect_location(uint32_t uid, ltm::Where &msg);
        void collect_streams(uint32_t uid, ltm::What &msg);
        void collect_entities(uint32_t uid, ltm::What &msg);

        virtual ~PluginsManager();
    };
}

#endif // LTM_PLUGINS_MANAGER_H
