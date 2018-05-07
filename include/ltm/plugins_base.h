#ifndef LTM_PLUGINS_BASE_H
#define LTM_PLUGINS_BASE_H

#include <ltm/EmotionalRelevance.h>

namespace ltm {
    namespace plugin {

        class EmotionBase {
            public:
            virtual void initialize(const std::string& param_ns) = 0;
            virtual void register_episode(uint32_t uid) = 0;
            virtual void unregister_episode(uint32_t uid) = 0;
            virtual void collect(uint32_t uid, ltm::EmotionalRelevance& msg) = 0;
            virtual ~EmotionBase(){}

            protected:
            EmotionBase(){}
        };

        class LocationBase {
        public:
            virtual void initialize(double side_length) = 0;
            virtual bool get_location() = 0;
            virtual ~LocationBase(){}

        protected:
            LocationBase(){}
        };

        class StreamBase {
        public:
            virtual void initialize(double side_length) = 0;
            virtual bool get_stream() = 0;
            virtual ~StreamBase(){}

        protected:
            StreamBase(){}
        };

        class EntityBase {
        public:
            virtual void initialize(double side_length) = 0;
            virtual bool get_entity() = 0;
            virtual ~EntityBase(){}

        protected:
            EntityBase(){}
        };

    }
}

#endif //LTM_PLUGINS_BASE_H
