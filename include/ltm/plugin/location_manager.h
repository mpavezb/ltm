#ifndef LTM_PLUGIN_LOCATION_MANAGER_H
#define LTM_PLUGIN_LOCATION_MANAGER_H

#include <pluginlib/class_loader.h>
#include <ltm/plugin/location_base.h>
#include <ltm/util/parameter_server_wrapper.h>

namespace ltm {
    namespace plugin {

        class LocationManager {
        private:
            typedef boost::shared_ptr<ltm::plugin::LocationBase> PluginPtr;

            // class loader
            pluginlib::ClassLoader<ltm::plugin::LocationBase> *_plugin_loader;

            // plugin
            PluginPtr _plugin;
            std::string _plugin_class;
            bool _use_plugin;

            void load_plugin();

        public:
            LocationManager();
            virtual ~LocationManager();

            void collect(uint32_t uid, ltm::Where &msg);
            void register_episode(uint32_t uid);
            void unregister_episode(uint32_t uid);
            void reset();
        };

    }
}


#endif //LTM_PLUGIN_LOCATION_MANAGER_H
