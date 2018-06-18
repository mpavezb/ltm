#include <ltm/util/util.h>

namespace ltm {
    namespace util {

        std::string vector_to_str(const std::vector<std::string> &array) {
            std::vector<std::string>::const_iterator it;
            std::stringstream ss;
            ss << "[";
            for (it = array.begin(); it != array.end(); ++it) {
                ss << *it << ", ";
            }
            ss.seekp(-2, ss.cur);
            ss << "]";
            return ss.str();
        }

        void vector_merge(std::vector<std::string> &result, const std::vector<std::string> &source) {
            std::vector<std::string>::const_iterator it;
            for (it = source.begin(); it != source.end(); ++it) {

                if (std::find(result.begin(), result.end(), *it) == result.end()) {
                    result.push_back(*it);
                }

            }
            std::sort(result.begin(), result.end());
        }

        void uid_vector_merge(std::vector<uint32_t> &result, const std::vector<uint32_t> &source) {
            std::vector<uint32_t>::const_iterator it;
            for (it = source.begin(); it != source.end(); ++it) {
                if (std::find(result.begin(), result.end(), *it) == result.end()) {
                    result.push_back(*it);
                }
            }
            std::sort(result.begin(), result.end());
        }

    }
}