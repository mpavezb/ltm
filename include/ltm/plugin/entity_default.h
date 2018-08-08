#ifndef LTM_PLUGIN_ENTITY_DEFAULT_H
#define LTM_PLUGIN_ENTITY_DEFAULT_H

#include <ltm/plugin/entity_ros.h>

namespace ltm {
    namespace plugin {

        // TODO: modificar nomenclatura: MsgType, SrvType
        template<class EntityMsg, class EntitySrv>
        class EntityDefault : public ltm::plugin::EntityROS<EntityMsg, EntitySrv> {


        };
    }
}

#endif //LTM_PLUGIN_ENTITY_DEFAULT_H
