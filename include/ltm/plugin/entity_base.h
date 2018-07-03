#ifndef LTM_ENTITY_BASE_H
#define LTM_ENTITY_BASE_H

#include <ros/time.h>
#include <ltm/Episode.h>
#include <ltm/QueryServer.h>
#include <ltm/db/types.h>

namespace ltm {
    namespace plugin {

        class EntityBase {
        public:
            EntityBase() {}
            virtual ~EntityBase() {}

            // LTM interface
            virtual std::string get_type() = 0;
            virtual void initialize(const std::string &param_ns, DBConnectionPtr ptr, std::string db_name) = 0;
            virtual void register_episode(uint32_t uid) = 0;
            virtual void unregister_episode(uint32_t uid) = 0;
            virtual void collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end) = 0;
            virtual void query(const std::string &json, ltm::QueryServer::Response &res) = 0;
            virtual void drop_db() = 0;
            virtual void reset(const std::string &db_name) = 0;
            virtual void append_status(std::stringstream &status) = 0;
        };
    }
}

#endif //LTM_ENTITY_BASE_H
