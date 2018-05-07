#ifndef LTM_PARAMETERSERVERWRAPPER_H
#define LTM_PARAMETERSERVERWRAPPER_H

// C, C++
#include <cstdio> // for EOF
#include <string>
#include <sstream>
#include <vector>

// ROS
#include <ros/ros.h>

namespace ltm {

    class ParameterServerWrapper {

    private:
        ros::NodeHandle priv;
        std::string prefix;

    public:
        ParameterServerWrapper(std::string name = "~");

        virtual ~ParameterServerWrapper();

        bool getParameter(std::string key, int &parameter, int default_value);

        bool getParameter(std::string key, bool &parameter, bool default_value);

        bool getParameter(std::string key, float &parameter, float default_value);

        bool getParameter(std::string key, double &parameter, double default_value);

        bool getParameter(std::string key, std::string &parameter, std::string default_value);

        bool getParameter(std::string key, std::vector<std::string> &parameter, std::vector<std::string> default_value);
    };

    inline ParameterServerWrapper::ParameterServerWrapper(std::string name) {
        priv = ros::NodeHandle(name);
        prefix = "(rosparam): ";
    }

    inline ParameterServerWrapper::~ParameterServerWrapper() {}

    inline bool ParameterServerWrapper::getParameter(std::string key, int &parameter, int default_value) {

        bool ret_val;
        if (priv.getParam(key, parameter)) {

            ROS_INFO_STREAM(prefix << "using custom '" << key << "': '" << parameter << "'");
            ret_val = true;

        } else {

            parameter = default_value;
            ROS_WARN_STREAM(prefix << "using default '" << key << "': '" << parameter << "'");
            ret_val = false;
        }

        priv.setParam(key, parameter);
        return ret_val;
    }

    inline bool ParameterServerWrapper::getParameter(std::string key, bool &parameter, bool default_value) {

        bool ret_val;
        if (priv.getParam(key, parameter)) {

            ROS_INFO_STREAM(prefix << "using custom '" << key << "': '" << parameter << "'");
            ret_val = true;

        } else {

            parameter = default_value;
            ROS_WARN_STREAM(prefix << "using default '" << key << "': '" << parameter << "'");
            ret_val = false;
        }

        priv.setParam(key, parameter);
        return ret_val;
    }

    inline bool ParameterServerWrapper::getParameter(std::string key, float &parameter, float default_value) {

        bool ret_val;
        double d_parameter;
        if (priv.getParam(key, d_parameter)) {

            parameter = (float) d_parameter;
            ROS_INFO_STREAM(prefix << "using custom '" << key << "': '" << parameter << "'");
            ret_val = true;

        } else {

            parameter = default_value;
            ROS_WARN_STREAM(prefix << "using default '" << key << "': '" << parameter << "'");
            ret_val = false;
        }
        priv.setParam(key, parameter);
        return ret_val;
    }

    inline bool ParameterServerWrapper::getParameter(std::string key, double &parameter, double default_value) {

        bool ret_val;
        if (priv.getParam(key, parameter)) {

            ROS_INFO_STREAM(prefix << "using custom '" << key << "': '" << parameter << "'");
            ret_val = true;

        } else {

            parameter = default_value;
            ROS_WARN_STREAM(prefix << "using default '" << key << "': '" << parameter << "'");
            ret_val = false;
        }
        priv.setParam(key, parameter);
        return ret_val;
    }

    inline bool
    ParameterServerWrapper::getParameter(std::string key, std::string &parameter, std::string default_value) {

        bool ret_val;
        if (priv.getParam(key, parameter)) {

            ROS_INFO_STREAM(prefix << "using custom '" << key << "': '" << parameter << "'");
            ret_val = true;

        } else {

            parameter = default_value;
            ROS_WARN_STREAM(prefix << "using default '" << key << "': '" << parameter << "'");
            ret_val = false;
        }
        priv.setParam(key, parameter);
        return ret_val;
    }

    inline bool
    ParameterServerWrapper::getParameter(std::string key, std::vector<std::string> &parameter,
                                         std::vector<std::string> default_value) {

        bool ret_val;
        if (priv.getParam(key, parameter)) {

            ROS_INFO_STREAM(
                    " - using custom string list for '" << key << "' which has (" << parameter.size() << ") points");
            ret_val = true;

        } else {

            parameter = default_value;
            ROS_WARN_STREAM(
                    " - using default string list for '" << key << "' which has (" << parameter.size() << ") points");
            ret_val = false;
        }
        priv.setParam(key, parameter);
        return ret_val;
    }

} /* namespace ltm */

#endif //LTM_PARAMETERSERVERWRAPPER_H
