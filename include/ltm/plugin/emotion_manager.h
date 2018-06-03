#ifndef LTM_PLUGIN_EMOTION_MANAGER_H
#define LTM_PLUGIN_EMOTION_MANAGER_H

#include <pluginlib/class_loader.h>
#include <ltm/plugin/emotion_base.h>
#include <ltm/parameter_server_wrapper.h>

namespace ltm {
    namespace plugin {

        class EmotionManager {
        private:
            typedef boost::shared_ptr<ltm::plugin::EmotionBase> PluginPtr;

            // class loader
            pluginlib::ClassLoader<ltm::plugin::EmotionBase> *_plugin_loader;

            // plugin
            PluginPtr _plugin;
            std::string _plugin_class;
            bool _use_plugin;

            void load_plugin();

        public:
            EmotionManager();

            virtual ~EmotionManager();

            void collect(uint32_t uid, ltm::EmotionalRelevance &msg);

            void register_episode(uint32_t uid);

            void unregister_episode(uint32_t uid);
        };
    }
}

#endif //LTM_PLUGIN_EMOTION_MANAGER_H
