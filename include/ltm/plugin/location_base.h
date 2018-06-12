#ifndef LTM_LOCATION_BASE_H
#define LTM_LOCATION_BASE_H

#include <ltm/Episode.h>

namespace ltm {
    namespace plugin {

        class LocationBase {
        public:
            virtual void initialize(const std::string &param_ns) = 0;

            virtual void register_episode(uint32_t uid) = 0;

            virtual void unregister_episode(uint32_t uid) = 0;

            virtual void collect(uint32_t uid, ltm::Where &msg) = 0;

            virtual void reset() = 0;

            virtual ~LocationBase() {}

        protected:
            LocationBase() {}
        };
    }
}

#endif //LTM_LOCATION_BASE_H
