#include <ltm/plugins_manager.h>

namespace ltm {

    PluginsManager::PluginsManager() {

    }

    PluginsManager::~PluginsManager() {}


    void PluginsManager::setup() {
        load_plugins();
    }

    void PluginsManager::load_plugins() {

        pluginlib::ClassLoader<ltm::plugin::EmotionBase> emotion_loader("ltm", "ltm::plugin::EmotionBase");
        pluginlib::ClassLoader<ltm::plugin::LocationBase> location_loader("ltm", "ltm::plugin::LocationBase");
        pluginlib::ClassLoader<ltm::plugin::StreamBase> stream_loader("ltm", "ltm::plugin::StreamBase");
        pluginlib::ClassLoader<ltm::plugin::EntityBase> entity_loader("ltm", "ltm::plugin::EntityBase");

        try {

            boost::shared_ptr<ltm::plugin::EmotionBase> emotion_pl = emotion_loader.createInstance("ltm_samples::EmotionPlugin");
            emotion_pl->initialize(10.0);

            boost::shared_ptr<ltm::plugin::LocationBase> location_pl = location_loader.createInstance("ltm_samples::LocationPlugin");
            location_pl->initialize(10.0);

            boost::shared_ptr<ltm::plugin::StreamBase> image_stream_pl = stream_loader.createInstance("ltm_samples::ImageStreamPlugin");
            image_stream_pl->initialize(10.0);

            boost::shared_ptr<ltm::plugin::EntityBase> people_entity_pl = entity_loader.createInstance("ltm_samples::PeopleEntityPlugin");
            people_entity_pl->initialize(10.0);

            boost::shared_ptr<ltm::plugin::EntityBase> objects_entity_pl = entity_loader.createInstance("ltm_samples::ObjectsEntityPlugin");
            objects_entity_pl->initialize(10.0);

            boost::shared_ptr<ltm::plugin::EntityBase> robot_entity_pl = entity_loader.createInstance("ltm_samples::RobotEntityPlugin");
            robot_entity_pl->initialize(10.0);

            boost::shared_ptr<ltm::plugin::EntityBase> location_entity_pl = entity_loader.createInstance("ltm_samples::LocationEntityPlugin");
            location_entity_pl->initialize(10.0);

        }
        catch(pluginlib::PluginlibException& ex) {
            ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }

    }

}
