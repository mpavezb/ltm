#include <ltm/plugin/emotion_manager.h>

namespace ltm {
    namespace plugin {

        EmotionManager::EmotionManager() {
            _plugin_loader = new pluginlib::ClassLoader<ltm::plugin::EmotionBase>("ltm", "ltm::plugin::EmotionBase");

            // ROS Parameters
            ltm::ParameterServerWrapper psw;
            psw.getParameter("plugins/emotion/class", _plugin_class, "");

            // load
            _use_plugin = false;
            load_plugin();
        }

        EmotionManager::~EmotionManager() {
            _plugin.reset();
            delete _plugin_loader;
        }

        void EmotionManager::load_plugin() {
            // valid class
            if (_plugin_class == "") {
                ROS_WARN_STREAM("LTM Emotion plugin class is empty. Deactivating the plugin.");
                return;
            }

            // load plugin
            try {
                _plugin = _plugin_loader->createInstance(_plugin_class);
            } catch (pluginlib::PluginlibException &ex) {
                ROS_WARN_STREAM(
                        "The LTM emotion plugin of class <" << _plugin_class << "> failed to load. Error: " << ex.what());
                return;
            }

            // initialize
            try {
                _plugin->initialize("plugins/emotion/");
            } catch (...) {
                ROS_WARN_STREAM("Couldn't initialize the LTM emotion plugin of class <" << _plugin_class << ">.");
                return;
            }
            ROS_INFO_STREAM("The LTM emotion plugin of class <" << _plugin_class << "> was successfully loaded.");
            _use_plugin = true;

        }

        void EmotionManager::collect(uint32_t uid, ltm::EmotionalRelevance &msg) {
            if (_use_plugin) _plugin->collect(uid, msg);
        }

        void EmotionManager::register_episode(uint32_t uid) {
            if (_use_plugin) _plugin->register_episode(uid);
        }

        void EmotionManager::unregister_episode(uint32_t uid) {
            if (_use_plugin) _plugin->unregister_episode(uid);
        }

        void EmotionManager::reset() {
            if (_use_plugin) _plugin->reset();
        }

    }
}
