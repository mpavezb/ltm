#ifndef LTM_PLUGIN_ENTITY_UTIL_H
#define LTM_PLUGIN_ENTITY_UTIL_H

#include <sensor_msgs/Image.h>
#include <ros/time.h>
#include <ltm/Date.h>
#include <ltm/EntityLog.h>
#include <vector>
#include <string>

namespace ltm {
    namespace plugin {
        namespace entity {

            inline std::string build_log_vector(const std::vector<std::string> &v) {
                std::string s = "[";
                std::vector<std::string>::const_iterator it;
                for (it = v.begin(); it != v.end(); ++it) {
                    s += *it + ", ";
                }
                s = (s.size() > 2) ? s = s.substr(0, s.size()-2) + "]" : "[]";
                return s;
            }

            template <typename T> bool field_equals(const T &A, const T &B);
            template <typename T> void update_field(ltm::EntityLog& log, const std::string &field, T &curr_e, T &log_e,  const T &new_e, const T &null_e);
        }
    }
}

#include <ltm/plugin/impl/entity_util_impl.hxx>

#endif //LTM_PLUGIN_ENTITY_UTIL_H
