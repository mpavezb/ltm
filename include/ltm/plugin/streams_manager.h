#ifndef LTM_PLUGIN_STREAMS_MANAGER_H
#define LTM_PLUGIN_STREAMS_MANAGER_H

#include <pluginlib/class_loader.h>
#include <ltm/plugin/stream_base.h>
#include <ltm/parameter_server_wrapper.h>

namespace ltm {
    namespace plugin {

        class StreamsManager {
        private:
            typedef boost::shared_ptr<ltm::plugin::StreamBase> PluginPtr;

            // class loader
            pluginlib::ClassLoader<ltm::plugin::StreamBase> *_plugin_loader;

            // plugins
            std::vector<PluginPtr> _plugins;
            std::vector<std::string> _plugin_classes;
            bool _use_plugins;

            DBConnectionPtr _conn;
            std::string _db_name;

            bool load_plugin(std::string plugin_class);

        public:
            StreamsManager(DBConnectionPtr ptr, std::string db_name);
            virtual ~StreamsManager();

            void collect(uint32_t uid, ltm::What &msg, ros::Time start, ros::Time end);
            void register_episode(uint32_t uid);
            void unregister_episode(uint32_t uid);
            void drop_db();
            void append_status(std::stringstream &status);
        };
    }
}

#endif //LTM_PLUGIN_STREAMS_MANAGER_H
