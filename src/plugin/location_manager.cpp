#include <ltm/plugin/location_manager.h>

namespace ltm {
    namespace plugin {

        LocationManager::LocationManager() {
            _plugin_loader = new pluginlib::ClassLoader<ltm::plugin::LocationBase>("ltm", "ltm::plugin::LocationBase");

            // ROS Parameters
            ltm::ParameterServerWrapper psw;
            psw.getParameter("plugins/location/class", _plugin_class, "");

            // load
            _use_plugin = false;
            load_plugin();
        }

        LocationManager::~LocationManager() {
            _plugin.reset();
            delete _plugin_loader;
        }

        void LocationManager::load_plugin() {
            // valid class
            if (_plugin_class == "") {
                ROS_WARN_STREAM("LTM Location plugin class is empty. Deactivating the plugin.");
                return;
            }

            // load plugin
            try {
                _plugin = _plugin_loader->createInstance(_plugin_class);
            } catch (pluginlib::PluginlibException &ex) {
                ROS_WARN_STREAM(
                        "The LTM location plugin of class <" << _plugin_class << "> failed to load. Error: " << ex.what());
                return;
            }

            // initialize
            try {
                _plugin->initialize("plugins/location/");
            } catch (...) {
                ROS_WARN_STREAM("Couldn't initialize the LTM location plugin of class <" << _plugin_class << ">.");
                return;
            }
            ROS_INFO_STREAM("The LTM location plugin of class <" << _plugin_class << "> was successfully loaded.");
            _use_plugin = true;
        }

        void LocationManager::collect(uint32_t uid, ltm::Where &msg) {
            if (_use_plugin) _plugin->collect(uid, msg);
        }

        void LocationManager::register_episode(uint32_t uid) {
            if (_use_plugin) _plugin->register_episode(uid);
        }

        void LocationManager::unregister_episode(uint32_t uid) {
            if (_use_plugin) _plugin->unregister_episode(uid);
        }
    }
}