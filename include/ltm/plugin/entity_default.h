#ifndef LTM_PLUGIN_ENTITY_DEFAULT_H
#define LTM_PLUGIN_ENTITY_DEFAULT_H

#include <ltm/plugin/entity_ros.h>

namespace ltm {
    namespace plugin {

        template<class EntityType, class EntitySrv>
        class EntityDefault : public ltm::plugin::EntityROS<EntityType, EntitySrv> {


        };
    }
}

#endif //LTM_PLUGIN_ENTITY_DEFAULT_H
