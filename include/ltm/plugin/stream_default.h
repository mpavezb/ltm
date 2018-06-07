#ifndef LTM_PLUGIN_STREAM_DEFAULT_H
#define LTM_PLUGIN_STREAM_DEFAULT_H

#include <ltm/plugin/stream_ros.h>

namespace ltm {
    namespace plugin {

        template<class StreamType, class StreamSrv>
        class StreamDefault : public ltm::plugin::StreamROS<StreamType, StreamSrv> {


        };
    }
}

#endif //LTM_PLUGIN_STREAM_DEFAULT_H
