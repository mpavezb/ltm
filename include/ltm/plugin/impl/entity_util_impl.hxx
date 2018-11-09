#ifndef LTM_PLUGIN_ENTITY_UTIL_HXX
#define LTM_PLUGIN_ENTITY_UTIL_HXX

#include <ltm/plugin/entity_util.h>


namespace ltm {
    namespace plugin {
        namespace entity {

            template <typename T>
            bool field_equals(const T &A, const T &B) {
                return A == B;
            }

            //explicit specialization for ltm::Date
            template <>
            inline bool field_equals<ltm::Date>(const ltm::Date &A, const ltm::Date &B) {
                return A.day == B.day && A.month == B.month && A.year == B.year;
            }

            //explicit specialization for ros::Time
            template <>
            inline bool field_equals<ros::Time>(const ros::Time &A, const ros::Time &B) {
                return A.sec == B.sec && A.nsec == B.nsec;
            }

            //explicit specialization for sensor_msgs::Image
            template <>
            inline bool field_equals<sensor_msgs::Image>(const sensor_msgs::Image &A, const sensor_msgs::Image &B) {
                return A.header.stamp.sec == B.header.stamp.sec
                       && A.header.stamp.nsec == B.header.stamp.nsec
                       && A.header.frame_id == B.header.frame_id
                       && A.header.seq == B.header.seq
                       && A.height == B.height
                       && A.width == B.width
                       && A.step == B.step;
            }

            template <typename T>
            void update_field(ltm::EntityLog &log, const std::string &field, T &curr_e,
                              T &log_e, const T &new_e, const T &null_e) {
                if (field_equals<T>(new_e, null_e)) {            // new field is null
                    if (field_equals<T>(curr_e, null_e)) {       // - and curr field is null     ---> NO CHANGES
                        log_e = null_e;
                    } else {                                     // - and curr field is not null ---> XXX REMOVE FIELD XXX
                        log_e = null_e;                          // KEEP WITHOUT CHANGES!.
                        // curr_e = null_e;
                        // log_e = new_e;
                        // log.removed_f.push_back(field);
                    }
                } else {                                         // new field is not null
                    if (field_equals<T>(curr_e, null_e)) {       // - and curr field is null     ---> NEW FIELD
                        curr_e = new_e;
                        log_e = new_e;
                        log.new_f.push_back(field);
                    } else if (field_equals<T>(curr_e, new_e)) { // - and curr field is the same ---> NO CHANGES
                        log_e = null_e;
                    } else {                                     // - and fields are different   ---> UPDATE FIELD
                        curr_e = new_e;
                        log_e = new_e;
                        log.updated_f.push_back(field);
                    }
                }
            }

        }
    }
}

#endif //LTM_PLUGIN_ENTITY_UTIL_HXX
