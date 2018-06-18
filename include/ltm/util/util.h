#ifndef LTM_UTIL_H
#define LTM_UTIL_H

#include <ltm/Episode.h>
#include <vector>
#include <set>
#include <string>
#include <iostream>
#include <algorithm>

namespace ltm {
    namespace util {

        void uid_vector_merge(std::vector<uint32_t> &result, const std::vector<uint32_t> &source);

        void vector_merge(std::vector<std::string> &result, const std::vector<std::string> &source);

        std::string vector_to_str(const std::vector<std::string> &array);

    }
}

#endif //LTM_UTIL_H
