#include <ltm/plugin/entities_manager.h>

namespace ltm {
    namespace plugin {

        EntitiesManager::EntitiesManager(DBConnectionPtr ptr, std::string db_name) {
            _plugin_loader = new pluginlib::ClassLoader<ltm::plugin::EntityBase>("ltm", "ltm::plugin::EntityBase");

            // setup DB
            _conn = ptr;
            _db_name = db_name;

            // Get entity plugin names from ROS Parameter server
            ltm::ParameterServerWrapper psw;
            std::vector<std::string> plugin_names;
            psw.getParameter("plugins/entities/include", plugin_names, plugin_names);

            // Load each declared plugin
            std::vector<std::string>::const_iterator it;
            for (it = plugin_names.begin(); it != plugin_names.end(); ++it) {
                ROS_INFO_STREAM("Loading Entity plugin named: " << *it);
                load_plugin(*it);
            }

            // there are registered plugins!
            _use_plugins = !_plugins.empty();
        }

        EntitiesManager::~EntitiesManager() {
            std::vector<PluginPtr>::iterator it;
            for (it = _plugins.begin(); it != _plugins.end(); ++it) {
                it->reset();
            }
            delete _plugin_loader;
        }

        bool EntitiesManager::load_plugin(std::string plugin_name) {
            ParameterServerWrapper psw;
            std::string param_ns = "plugins/entities/" + plugin_name + "/";

            // plugin class
            std::string plugin_class;
            psw.getParameter(param_ns + "class", plugin_class, "");
            if (plugin_class == "") {
                ROS_WARN_STREAM("LTM Entity plugin class for name (" << plugin_name << ") is empty. Won't use this plugin.");
                return false;
            }

            // load plugin
            PluginPtr pl_ptr;
            try {
                pl_ptr = _plugin_loader->createInstance(plugin_class);
            } catch (pluginlib::PluginlibException &ex) {
                ROS_WARN_STREAM("The LTM Entity plugin of name (" << plugin_name << ") and class <" << plugin_class
                                                                  << "> failed to load. Error: " << ex.what());
                pl_ptr.reset();
                return false;
            }

            // initialize plugin
            try {
                pl_ptr->initialize(param_ns, _conn, _db_name);
            } catch (std::exception& e) {
                ROS_WARN_STREAM(
                        "Couldn't initialize the LTM Entity plugin of name ("
                                << plugin_name << ") and class <" << plugin_class << ">. Because: " << e.what());
                pl_ptr.reset();
                return false;
            }

            // save it
            ROS_INFO_STREAM("The LTM Entity plugin of name (" << plugin_name << ") and class <" << plugin_class
                                                              << "> was successfully loaded.");
            _plugins.push_back(pl_ptr);
            return true;
        }

        void EntitiesManager::collect(uint32_t uid, ltm::What &msg, ros::Time start, ros::Time end) {
            if (_use_plugins) {
                std::vector<PluginPtr>::iterator it;
                for (it = _plugins.begin(); it != _plugins.end(); ++it) {
                    (*it)->collect(uid, msg, start, end);
                }
            }
        }

        void EntitiesManager::register_episode(uint32_t uid) {
            if (_use_plugins) {
                std::vector<PluginPtr>::iterator it;
                for (it = _plugins.begin(); it != _plugins.end(); ++it) {
                    (*it)->register_episode(uid);
                }
            }
        }

        void EntitiesManager::unregister_episode(uint32_t uid) {

        }

        void EntitiesManager::drop_db() {
            if (_use_plugins) {
                std::vector<PluginPtr>::iterator it;
                for (it = _plugins.begin(); it != _plugins.end(); ++it) {
                    (*it)->drop_db();
                }
            }
        }

        void EntitiesManager::append_status(std::stringstream &status) {
            if (_use_plugins) {
                std::vector<PluginPtr>::iterator it;
                for (it = _plugins.begin(); it != _plugins.end(); ++it) {
                    status << " - ";
                    (*it)->append_status(status);
                    status << std::endl;
                }
            }
        }
    }
}