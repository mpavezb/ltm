#ifndef LTM_PLUGINS_MANAGER_H
#define LTM_PLUGINS_MANAGER_H

#include <pluginlib/class_loader.h>
#include <ltm/plugins_base.h>

namespace ltm {

    class PluginsManager {
    public:
        PluginsManager();
        virtual ~PluginsManager();

        void setup();
        void load_plugins();
    };
}

#endif // LTM_PLUGINS_MANAGER_H
