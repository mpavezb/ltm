#include <ltm/plugin/streams_manager.h>

namespace ltm {
    namespace plugin {

        StreamsManager::StreamsManager(DBConnectionPtr ptr, std::string db_name) {
            _plugin_loader = new pluginlib::ClassLoader<ltm::plugin::StreamBase>("ltm", "ltm::plugin::StreamBase");

            // setup DB
            _conn = ptr;
            _db_name = db_name;

            // Get stream plugin names from ROS Parameter server
            ltm::ParameterServerWrapper psw;
            std::vector<std::string> plugin_names;
            psw.getParameter("plugins/streams/include", plugin_names, plugin_names);

            // Load each declared plugin
            std::vector<std::string>::const_iterator it;
            for (it = plugin_names.begin(); it != plugin_names.end(); ++it) {
                ROS_INFO_STREAM("Loading Stream plugin named: " << *it);
                load_plugin(*it);
            }

            // there are registered plugins!
            _use_plugins = !_plugins.empty();
        }

        StreamsManager::~StreamsManager() {
            std::vector<PluginPtr>::iterator it;
            for (it = _plugins.begin(); it != _plugins.end(); ++it) {
                it->reset();
            }
            delete _plugin_loader;
        }

        bool StreamsManager::load_plugin(std::string plugin_name) {
            ParameterServerWrapper psw;

            // plugin class
            std::string plugin_class;
            psw.getParameter("plugins/streams/" + plugin_name + "/class", plugin_class, "");
            if (plugin_class == "") {
                ROS_WARN_STREAM("LTM Stream plugin class for name (" << plugin_name << ") is empty. Won't use this plugin.");
                return false;
            }

            // load plugin
            PluginPtr pl_ptr;
            try {
                pl_ptr = _plugin_loader->createInstance(plugin_class);
            } catch (pluginlib::PluginlibException &ex) {
                ROS_WARN_STREAM("The LTM Stream plugin of name (" << plugin_name << ") and class <" << plugin_class
                                                                  << "> failed to load. Error: " << ex.what());
                return false;
            }

            // initialize
            try {
                pl_ptr->initialize("plugins/streams/" + plugin_name + "/", _conn, _db_name);
            } catch (std::exception& e) {
                ROS_WARN_STREAM(
                        "Couldn't initialize the LTM Stream plugin of name ("
                                << plugin_name << ") and class <" << plugin_class << ">. Because: " << e.what());
                return false;
            }

            // save it
            ROS_INFO_STREAM("The LTM Stream plugin of name (" << plugin_name << ") and class <" << plugin_class
                                                              << "> was successfully loaded.");
            _plugins.push_back(pl_ptr);

            return true;
        }

        void StreamsManager::collect(uint32_t uid, ltm::What &msg, ros::Time start, ros::Time end) {
            if (_use_plugins) {
                std::vector<PluginPtr>::iterator it;
                for (it = _plugins.begin(); it != _plugins.end(); ++it) {
                    (*it)->collect(uid, msg, start, end);
                }
            }
        }

        void StreamsManager::register_episode(uint32_t uid) {
            if (_use_plugins) {
                std::vector<PluginPtr>::iterator it;
                for (it = _plugins.begin(); it != _plugins.end(); ++it) {
                    (*it)->register_episode(uid);
                }
            }
        }

        void StreamsManager::unregister_episode(uint32_t uid) {

        }
    }
}