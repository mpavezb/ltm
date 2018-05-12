#ifndef LTM_PLUGINS_BASE_H
#define LTM_PLUGINS_BASE_H

#include <ltm/Episode.h>
#include <ltm/db_manager.h>

namespace ltm {
    namespace plugin {

        class EmotionBase {
        public:
            virtual void initialize(const std::string &param_ns) = 0;
            virtual void register_episode(uint32_t uid) = 0;
            virtual void unregister_episode(uint32_t uid) = 0;
            virtual void collect(uint32_t uid, ltm::EmotionalRelevance &msg) = 0;

            virtual ~EmotionBase() {}

        protected:
            EmotionBase() {}
        };

        class LocationBase {
        public:
            virtual void initialize(const std::string &param_ns) = 0;
            virtual void register_episode(uint32_t uid) = 0;
            virtual void unregister_episode(uint32_t uid) = 0;
            virtual void collect(uint32_t uid, ltm::Where &msg) = 0;

            virtual ~LocationBase() {}

        protected:
            LocationBase() {}
        };

        class StreamBase {
        public:
            virtual void initialize(const std::string &param_ns, DBConnectionPtr ptr, std::string db_name) = 0;
            virtual void register_episode(uint32_t uid) = 0;
            virtual void unregister_episode(uint32_t uid) = 0;
            virtual void collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end) = 0;
            virtual void degrade(uint32_t uid) = 0;
            virtual std::string get_type() = 0;
            virtual std::string get_collection_name() = 0;

            // - - - - - - - - -  - DB API - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            virtual bool remove(uint32_t uid) = 0;
            virtual int count() = 0;
            virtual bool has(int uid) = 0;
            virtual bool is_reserved(int uid) = 0;
            virtual bool drop_db() = 0;
            virtual void setup_db() = 0;

            virtual ~StreamBase() {}

        protected:
            StreamBase() {}
        };

        class EntityBase {
        public:
            virtual void initialize(const std::string &param_ns, DBConnectionPtr ptr, std::string db_name) = 0;
            virtual void register_episode(uint32_t uid) = 0;
            virtual void unregister_episode(uint32_t uid) = 0;
            virtual void collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end) = 0;
            virtual std::string get_type() = 0;
            virtual std::string get_collection_name() = 0;

            // - - - - - - - - -  - DB API - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            virtual bool remove(uint32_t uid) = 0;
            virtual int count() = 0;
            virtual bool has(int uid) = 0;
            virtual bool is_reserved(int uid) = 0;
            virtual bool drop_db() = 0;
            virtual void setup_db() = 0;

            virtual ~EntityBase() {}

        protected:
            EntityBase() {}
        };

    }
}

#endif //LTM_PLUGINS_BASE_H
