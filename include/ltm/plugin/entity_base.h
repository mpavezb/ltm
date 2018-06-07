#ifndef LTM_ENTITY_BASE_H
#define LTM_ENTITY_BASE_H

#include <ros/time.h>
#include <ltm/Episode.h>
#include <ltm/db/types.h>

namespace ltm {
    namespace plugin {

        class EntityBase {
        public:
            EntityBase() {}
            virtual ~EntityBase() {}

            // LTM interface
            virtual void initialize(const std::string &param_ns, DBConnectionPtr ptr, std::string db_name) = 0;
            virtual void register_episode(uint32_t uid) = 0;
            virtual void unregister_episode(uint32_t uid) = 0;
            virtual void collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end) = 0;

        };
    }
}

#endif //LTM_ENTITY_BASE_H
