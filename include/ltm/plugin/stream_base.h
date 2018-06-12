#ifndef LTM_STREAM_BASE_H
#define LTM_STREAM_BASE_H

#include <ros/time.h>
#include <ltm/db/types.h>
#include <ltm/Episode.h>

namespace ltm {
    namespace plugin {

        class StreamBase {
        public:
            StreamBase() {}
            virtual ~StreamBase() {}

            // LTM interface
            virtual void initialize(const std::string &param_ns, DBConnectionPtr ptr, std::string db_name) = 0;
            virtual void register_episode(uint32_t uid) = 0;
            virtual void unregister_episode(uint32_t uid) = 0;
            virtual void collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end) = 0;
            virtual void degrade(uint32_t uid) = 0;
            virtual void drop_db() = 0;
            virtual void append_status(std::stringstream &status) = 0;
        };
    }
}

#endif //LTM_STREAM_BASE_H
