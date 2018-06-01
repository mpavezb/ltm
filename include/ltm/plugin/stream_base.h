#ifndef LTM_STREAM_BASE_H
#define LTM_STREAM_BASE_H

#include <ros/time.h>
#include <ltm/Episode.h>
#include <ltm/db/types.h>
#include <ltm/db/stream_collection.h>

namespace ltm {
    namespace plugin {

        /**
         * Requirements:
         *
         * - Server API
         *   - initialize
         *   - register    <----- OK
         *   - unregister  <----- OK
         *   - collect
         *   - degrade     <-USER
         *   - type        <----- OK
         *   - collection  <----- OK
         *
         * - ROS services
         *   - drop_db
         *   - add
         *   - get
         *   - update
         *   - delete
         *
         * - Collection Management
         *   - count
         *   - remove
         *   - has
         *   - get
         *   - is_reserved
         *   - drop         <-----
         *   - setup        <-----
         *
         *
         * <----- : can be generalized
         * <-USER : must be implemented by the user
         *
         */
        class StreamBase {
        private:


        public:
            virtual std::string get_type() = 0;
            virtual std::string get_collection_name() = 0;
            virtual void register_episode(uint32_t uid) = 0;
            virtual void unregister_episode(uint32_t uid) = 0;
            virtual bool is_reserved(int uid) = 0;
            virtual void setup_db() = 0;

            virtual void initialize(const std::string &param_ns, DBConnectionPtr ptr, std::string db_name) = 0;
            virtual void collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end) = 0;
            virtual void degrade(uint32_t uid) = 0;



            // - - - - - - - - -  - DB API - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            virtual bool remove(uint32_t uid) = 0;
            virtual int count() = 0;
            virtual bool has(int uid) = 0;
            virtual bool drop_db() = 0;

            virtual ~StreamBase() {}

        protected:
            StreamBase() {}

            virtual void subscribe() {};
            virtual void unsubscribe() {};
        };
    }
}

#endif //LTM_STREAM_BASE_H
