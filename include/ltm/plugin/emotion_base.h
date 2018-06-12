#ifndef LTM_EMOTION_BASE_H
#define LTM_EMOTION_BASE_H

#include <ltm/Episode.h>

namespace ltm {
    namespace plugin {

        class EmotionBase {
        public:
            virtual void initialize(const std::string &param_ns) = 0;

            virtual void register_episode(uint32_t uid) = 0;

            virtual void unregister_episode(uint32_t uid) = 0;

            virtual void collect(uint32_t uid, ltm::EmotionalRelevance &msg) = 0;

            virtual void reset() = 0;

            virtual ~EmotionBase() {}

        protected:
            EmotionBase() {}
        };
    }
}

#endif //LTM_EMOTION_BASE_H
