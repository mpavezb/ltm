#include <ltm/plugins_manager.h>
#include <ltm/parameter_server_wrapper.h>

namespace ltm {

    PluginsManager::PluginsManager() {
        _use_emotion_pl = false;
        _use_location_pl = false;
        _use_stream_pls = false;
        _use_entity_pls = false;

        // class loaders
        _emotion_loader = new pluginlib::ClassLoader<ltm::plugin::EmotionBase>("ltm", "ltm::plugin::EmotionBase");
        _location_loader = new pluginlib::ClassLoader<ltm::plugin::LocationBase>("ltm", "ltm::plugin::LocationBase");
        _stream_loader = new pluginlib::ClassLoader<ltm::plugin::StreamBase>("ltm", "ltm::plugin::StreamBase");
        _entity_loader = new pluginlib::ClassLoader<ltm::plugin::EntityBase>("ltm", "ltm::plugin::EntityBase");

    }

    PluginsManager::~PluginsManager() {
        _emotion_pl.reset();
        delete _emotion_loader;

        _location_pl.reset();
        delete _location_loader;

        std::vector<StreamPluginPtr>::iterator stream_it;
        for (stream_it = _stream_pls.begin(); stream_it != _stream_pls.end(); ++stream_it) {
            stream_it->reset();
        }
        delete _stream_loader;

        std::vector<EntityPluginPtr>::iterator entity_it;
        for (entity_it = _entity_pls.begin(); entity_it != _entity_pls.end(); ++entity_it) {
            entity_it->reset();
        }
        delete _entity_loader;
    }

    void PluginsManager::setup() {
        ParameterServerWrapper psw;

        // setup emotion plugin
        std::string emotion_class;
        psw.getParameter("plugins/emotion/class", emotion_class, "");
        load_emotion_plugin(emotion_class);

        // setup location plugin
        std::string location_class;
        psw.getParameter("plugins/location/class", location_class, "");
        load_location_plugin(location_class);

        // setup stream plugins
        std::vector<std::string> stream_names;
        psw.getParameter("plugins/streams/include", stream_names, stream_names);
        load_stream_plugins(stream_names);

        // setup entity plugins
        std::vector<std::string> entity_names;
        psw.getParameter("plugins/entities/include", entity_names, entity_names);
        load_entity_plugins(entity_names);
    }

    void PluginsManager::load_emotion_plugin(std::string plugin_class) {
        // valid class
        if (plugin_class == "") {
            ROS_WARN_STREAM("LTM Emotion plugin class is empty. Deactivating the plugin.");
            return;
        }

        // load plugin
        try {
            _emotion_pl = _emotion_loader->createInstance(plugin_class);
        } catch (pluginlib::PluginlibException &ex) {
            ROS_WARN_STREAM(
                    "The LTM emotion plugin of class <" << plugin_class << "> failed to load. Error: " << ex.what());
            return;
        }

        // initialize
        try {
            _emotion_pl->initialize("plugins/emotion/");
        } catch (...) {
            ROS_WARN_STREAM("Couldn't initialize the LTM emotion plugin of class <" << plugin_class << ">.");
            return;
        }
        ROS_INFO_STREAM("The LTM emotion plugin of class <" << plugin_class << "> was successfully loaded.");
        _use_emotion_pl = true;
    }

    void PluginsManager::load_location_plugin(std::string plugin_class) {
        // valid class
        if (plugin_class == "") {
            ROS_WARN_STREAM("LTM Location plugin class is empty. Deactivating the plugin.");
            return;
        }

        // load plugin
        try {
            _location_pl = _location_loader->createInstance(plugin_class);
        } catch (pluginlib::PluginlibException &ex) {
            ROS_WARN_STREAM(
                    "The LTM location plugin of class <" << plugin_class << "> failed to load. Error: " << ex.what());
            return;
        }

        // initialize
        try {
            _location_pl->initialize("plugins/location/");
        } catch (...) {
            ROS_WARN_STREAM("Couldn't initialize the LTM location plugin of class <" << plugin_class << ">.");
            return;
        }
        ROS_INFO_STREAM("The LTM location plugin of class <" << plugin_class << "> was successfully loaded.");
        _use_location_pl = true;
    }

    void PluginsManager::load_stream_plugins(const std::vector<std::string> &plugin_classes) {
        ParameterServerWrapper psw;
        std::vector<std::string>::const_iterator it;
        for (it = plugin_classes.begin(); it != plugin_classes.end(); ++it) {
            ROS_INFO_STREAM("Loading Stream plugin named: " << *it);
            std::string plugin_class;
            psw.getParameter("plugins/streams/" + *it + "/class", plugin_class, "");

            // valid class
            if (plugin_class == "") {
                ROS_WARN_STREAM("LTM Stream plugin class for name (" << *it << ") is empty. Won't use this plugin.");
                continue;
            }

            // load plugin
            StreamPluginPtr pl_ptr;
            try {
                pl_ptr = _stream_loader->createInstance(plugin_class);
            } catch (pluginlib::PluginlibException &ex) {
                ROS_WARN_STREAM("The LTM Stream plugin of name (" << *it << ") and class <" << plugin_class
                                                                  << "> failed to load. Error: " << ex.what());
                continue;
            }

            // initialize
            try {
                pl_ptr->initialize("plugin/streams/" + *it + "/");
            } catch (...) {
                ROS_WARN_STREAM(
                        "Couldn't initialize the LTM Stream plugin of name (" << *it << ") and class <" << plugin_class
                                                                              << ">.");
                continue;
            }

            // save it
            ROS_INFO_STREAM("The LTM Stream plugin of name (" << *it << ") and class <" << plugin_class
                                                              << "> was successfully loaded.");
            _stream_pls.push_back(pl_ptr);
        }

        // there are registered plugins!
        _use_stream_pls = !_stream_pls.empty();
    }

    void PluginsManager::load_entity_plugins(const std::vector<std::string> &plugin_classes) {
        ParameterServerWrapper psw;
        std::vector<std::string>::const_iterator it;
        for (it = plugin_classes.begin(); it != plugin_classes.end(); ++it) {
            ROS_INFO_STREAM("Loading Entity plugin named: " << *it);
            std::string plugin_class;
            psw.getParameter("plugins/entities/" + *it + "/class", plugin_class, "");

            // valid class
            if (plugin_class == "") {
                ROS_WARN_STREAM("LTM Entity plugin class for name (" << *it << ") is empty. Won't use this plugin.");
                continue;
            }

            // load plugin
            EntityPluginPtr pl_ptr;
            try {
                pl_ptr = _entity_loader->createInstance(plugin_class);
            } catch (pluginlib::PluginlibException &ex) {
                ROS_WARN_STREAM("The LTM Entity plugin of name (" << *it << ") and class <" << plugin_class
                                                                  << "> failed to load. Error: " << ex.what());
                continue;
            }

            // initialize
            try {
                pl_ptr->initialize("plugin/entities/" + *it + "/");
            } catch (...) {
                ROS_WARN_STREAM(
                        "Couldn't initialize the LTM Entity plugin of name (" << *it << ") and class <" << plugin_class
                                                                              << ">.");
                continue;
            }

            // save it
            ROS_INFO_STREAM("The LTM Entity plugin of name (" << *it << ") and class <" << plugin_class
                                                              << "> was successfully loaded.");
            _entity_pls.push_back(pl_ptr);
        }

        // there are registered plugins!
        _use_entity_pls = !_entity_pls.empty();
    }

    void PluginsManager::register_episode(uint32_t uid) {
        if (_use_emotion_pl) _emotion_pl->register_episode(uid);
        if (_use_location_pl) _location_pl->register_episode(uid);
        if (_use_stream_pls) {
            std::vector<StreamPluginPtr>::iterator stream_it;
            for (stream_it = _stream_pls.begin(); stream_it != _stream_pls.end(); ++stream_it) {
                (*stream_it)->register_episode(uid);
            }
        }
        if (_use_entity_pls) {
            std::vector<EntityPluginPtr>::iterator entity_it;
            for (entity_it = _entity_pls.begin(); entity_it != _entity_pls.end(); ++entity_it) {
                (*entity_it)->register_episode(uid);
            }
        }
    }

    void PluginsManager::unregister_episode(uint32_t uid) {
        if (_use_emotion_pl) _emotion_pl->unregister_episode(uid);
        if (_use_location_pl) _location_pl->unregister_episode(uid);
        if (_use_stream_pls) {
            std::vector<StreamPluginPtr>::iterator stream_it;
            for (stream_it = _stream_pls.begin(); stream_it != _stream_pls.end(); ++stream_it) {
                (*stream_it)->unregister_episode(uid);
            }
        }
        if (_use_entity_pls) {
            std::vector<EntityPluginPtr>::iterator entity_it;
            for (entity_it = _entity_pls.begin(); entity_it != _entity_pls.end(); ++entity_it) {
                (*entity_it)->unregister_episode(uid);
            }
        }
    }

    void PluginsManager::collect_emotion(uint32_t uid, ltm::EmotionalRelevance &msg) {
        if (_use_emotion_pl) _emotion_pl->collect(uid, msg);
    }

    void PluginsManager::collect_location(uint32_t uid, ltm::Where &msg) {
        if (_use_location_pl) _location_pl->collect(uid, msg);
    }

    void PluginsManager::collect_streams(uint32_t uid, ltm::What &msg) {
        if (_use_stream_pls) {
            std::vector<StreamPluginPtr>::iterator stream_it;
            for (stream_it = _stream_pls.begin(); stream_it != _stream_pls.end(); ++stream_it) {
                (*stream_it)->collect(uid, msg);
            }
        }
    }

    void PluginsManager::collect_entities(uint32_t uid, ltm::What &msg) {
        if (_use_entity_pls) {
            std::vector<EntityPluginPtr>::iterator entity_it;
            for (entity_it = _entity_pls.begin(); entity_it != _entity_pls.end(); ++entity_it) {
                (*entity_it)->collect(uid, msg);
            }
        }
    }
}
