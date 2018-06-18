
#include <ltm/util/util.h>
#include <ltm/util/geometry.h>
#include <ltm/db/episode_updater.h>

namespace ltm {
    namespace db {

        // =================================================================================================================
        // Plublic API
        // =================================================================================================================
        //

        void EpisodeUpdater::update_tree_init(ltm::Episode &node, EpisodeUpdateHelper &helper) {
            update_tree_tags_init(node);
            update_tree_info_init(node.info);
            update_tree_when_init(node.when);
            update_tree_where_init(node.where);
            update_tree_what_init(node.what);
            update_tree_relevance_init(node.relevance);
            helper.positions.clear();
        }

        bool EpisodeUpdater::update_tree_last(ltm::Episode &node, EpisodeUpdateHelper &helper) {
            bool result = true;
            result = result && update_tree_where_last(node.where, helper.positions);
            return result;
        }

        bool EpisodeUpdater::update_from_child(ltm::Episode &node, const ltm::Episode &child, EpisodeUpdateHelper &helper) {
            bool result = true;
            bool is_leaf = (child.type == ltm::Episode::LEAF);
            result = result && update_tree_tags(node, child);
            result = result && update_tree_info(node.info, child.info);
            result = result && update_tree_when(node.when, child.when);
            result = result && update_tree_where(node.where, child.where, is_leaf, node.uid, child.uid, helper.positions);
            result = result && update_tree_what(node.what, child.what);
            result = result && update_tree_relevance(node.relevance, child.relevance, is_leaf);
            return result;
        }

        // =================================================================================================================
        // Private API
        // =================================================================================================================
        //

        void EpisodeUpdater::update_tree_tags_init(Episode &node) {
            // do not clear own tags!.
            node.children_tags.clear();
        }

        bool EpisodeUpdater::update_tree_tags(Episode &node, const Episode &child) {
            ltm::util::vector_merge(node.children_tags, child.tags);
            ltm::util::vector_merge(node.children_tags, child.children_tags);
            ROS_DEBUG_STREAM(" ---> tags: " << ltm::util::vector_to_str(node.tags));
            ROS_DEBUG_STREAM(" ---> child tags: " << ltm::util::vector_to_str(node.children_tags));
            return true;
        }

        void EpisodeUpdater::update_tree_info_init(Info &node) {
            node.n_usages = 0;
            node.creation_date = ros::Time(0, 0);
        }

        bool EpisodeUpdater::update_tree_info(Info &node, const Info &child) {
            // TODO: DOC n_usages parent = SUM children
            int n_usages = node.n_usages;

            // keep last children information
            if (node.creation_date < child.creation_date) {
                node = child;
            }

            // and add n_usages
            node.n_usages += n_usages;
            return true;
        }

        void EpisodeUpdater::update_tree_when_init(When &node) {
            // TODO: DESIGN DECISION.. only update if children is not contained by the episode.

            // avoid default values
            if (node.start == ros::Time()) node.start = ros::Time::now();
            if (node.end == ros::Time()) node.start = ros::Time(0, 0);

            // By uncommenting this lines, only children WHEN values will be considered.
            // node.start = ros::Time::now();
            // node.end = ros::Time(0, 0);
        }

        bool EpisodeUpdater::update_tree_when(When &node, const When &child) {
            if (child.start < node.start) {
                node.start = child.start;
            }
            if (child.end > node.end) {
                node.end = child.end;
            }
            return true;
        }

        void EpisodeUpdater::update_tree_where_init(Where &node) {
            // node is never a leaf!

            // numerical info
            node.frame_id = "";
            node.map_name = "";
            node.position = geometry_msgs::Point();

            // semantic info
            node.location = "";
            node.area = "";

            // children information
            node.children_locations.clear();
            node.children_areas.clear();
            node.children_hull.clear();
        }

        bool EpisodeUpdater::update_tree_where_last(Where &node, std::vector<geometry_msgs::Point> &positions) {
            ROS_DEBUG_STREAM(" ---> where (pre hull): " << ltm::util::point_vector_to_str(positions));
            // returns ordered list
            ltm::util::convex_hull_2d(positions, node.children_hull);

            // this requires an ordered list
            node.position = ltm::util::polygon_centroid_2d(node.children_hull);
            ROS_DEBUG_STREAM(" ---> where.children_hull: " << ltm::util::point_vector_to_str(node.children_hull));
            ROS_DEBUG_STREAM(" ---> where.position: (" << node.position.x << ", " << node.position.y << ")");
            return true;
        }

        bool EpisodeUpdater::update_tree_where(Where &node, const Where &child, bool is_leaf, int node_uid, int child_uid, std::vector<geometry_msgs::Point> &positions) {
            if (is_leaf) {
                std::vector<std::string> child_location;
                child_location.push_back(child.location);
                ltm::util::vector_merge(node.children_locations, child_location);

                std::vector<std::string> child_area;
                child_area.push_back(child.area);
                ltm::util::vector_merge(node.children_areas, child_area);

                // just a point
                positions.push_back(child.position);
            } else {
                ltm::util::vector_merge(node.children_locations, child.children_locations);
                ltm::util::vector_merge(node.children_areas, child.children_areas);

                // join children hull + other points
                positions.reserve(positions.size() + child.children_hull.size());
                positions.insert(positions.end(), child.children_hull.begin(), child.children_hull.end());
            }

            // TODO: DOC: require same frame_id and map_name on a tree
            // frame_id and map_name must be the same for all children
            bool result = true;
            if (node.map_name == "" && node.frame_id == "") {
                // first time
                node.frame_id = child.frame_id;
                node.map_name = child.map_name;
            } else {
                // next children checks
                if (node.frame_id != child.frame_id) {
                    ROS_WARN_STREAM("UPDATE: parent (" << node_uid << ") where.frame_id {" << node.frame_id
                                                       << "} does not match child's (" << child_uid << ") {" << child.frame_id << "}.");
                    result = false;
                }
                if (node.map_name != child.map_name) {
                    ROS_WARN_STREAM("UPDATE: parent (" << node_uid << ") where.map_name {" << node.map_name
                                                       << "} does not match child's (" << child_uid << ") {" << child.map_name << "}.");
                    result = false;
                }
            }
            return result;
        }

        void EpisodeUpdater::update_tree_what_init(What &node) {
            node.entities.clear();
            node.streams.clear();
        }

        struct StreamRegisterCompare : public std::unary_function<ltm::StreamRegister, bool>
        {
            ltm::StreamRegister baseline;
            explicit StreamRegisterCompare(const ltm::StreamRegister &baseline) : baseline(baseline) {}
            bool operator() (const ltm::StreamRegister &arg) {
                return arg.uid == baseline.uid && arg.type == baseline.type;
            }
        };

        struct EntityRegisterCompare : public std::unary_function<ltm::EntityRegister, bool>
        {
            ltm::EntityRegister baseline;
            explicit EntityRegisterCompare(const ltm::EntityRegister &baseline) : baseline(baseline) {}
            bool operator() (const ltm::EntityRegister &arg) {
                return arg.uid == baseline.uid && arg.type == baseline.type;
            }
        };

        bool EpisodeUpdater::update_tree_what(What &node, const What &child) {
            bool result = true;
            result = result && this->update_tree_what_streams(node, child);
            result = result && this->update_tree_what_entities(node, child);
            return result;
        }

        bool EpisodeUpdater::update_tree_what_streams(What &node, const What &child) {
            // Append StreamRegister fields keeping unique values.
            std::vector<StreamRegister>::const_iterator s_it, sf_it;
            for (s_it = child.streams.begin(); s_it != child.streams.end(); ++s_it) {
                sf_it = std::find_if(node.streams.begin(), node.streams.end(), StreamRegisterCompare(*s_it));
                if (sf_it == node.streams.end()) {
                    node.streams.push_back(*s_it);
                }
            }
            return true;
        }

        bool EpisodeUpdater::update_tree_what_entities(What &node, const What &child) {
            // Add new EntityRegister fields and update log uids for existent registers.
            std::vector<EntityRegister>::const_iterator e_it;
            std::vector<EntityRegister>::iterator ef_it;
            for (e_it = child.entities.begin(); e_it != child.entities.end(); ++e_it) {
                ef_it = std::find_if(node.entities.begin(), node.entities.end(), EntityRegisterCompare(*e_it));
                if (ef_it == node.entities.end()) {
                    // new register
                    node.entities.push_back(*e_it);
                } else {
                    // already exists: merge
                    ltm::util::uid_vector_merge(ef_it->log_uids, e_it->log_uids);
                }
            }
            return true;
        }

        void EpisodeUpdater::update_tree_relevance_init(Relevance &node) {
            update_tree_relevance_historical_init(node.historical);
            update_tree_relevance_emotional_init(node.emotional);
        }

        bool EpisodeUpdater::update_tree_relevance(Relevance &node, const Relevance &child, bool is_leaf) {
            bool result = true;
            result = result && update_tree_relevance_historical(node.historical, child.historical);
            result = result && update_tree_relevance_emotional(node.emotional, child.emotional, is_leaf);
            return result;
        }

        void EpisodeUpdater::update_tree_relevance_historical_init(HistoricalRelevance &node) {
            node.value = 0;
        }

        bool EpisodeUpdater::update_tree_relevance_historical(HistoricalRelevance &node, const HistoricalRelevance &child) {
            if (child.value > node.value) {
                node = child;
            }
            return true;
        }

        void EpisodeUpdater::update_tree_relevance_emotional_init(EmotionalRelevance &node) {
            // init with ordered emotions: 0 to 7
            static const int arr[] = {0, 1, 2, 3, 4, 5, 6, 7};
            std::vector<int32_t> tmp(arr, arr + sizeof(arr) / sizeof(arr[0]));
            node.children_emotions.clear();
            node.children_emotions = tmp;

            static const float arr2[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            std::vector<float> tmp2(arr2, arr2 + sizeof(arr2) / sizeof(arr2[0]));
            node.children_values.clear();
            node.children_values = tmp2;

            // init global values with zeros
            node.emotion = EmotionalRelevance::JOY;
            node.value = 0.0;

            // TODO: DOC parent does not uses this information
            node.software = "";
            node.software_version = "";
            node.registered_emotions.clear();
            node.registered_values.clear();
        }

        bool EpisodeUpdater::update_tree_relevance_emotional(EmotionalRelevance &node, const EmotionalRelevance &child, bool is_leaf) {
            // TODO: DOC: children_emotions/values are keep ordered by emotion number
            // TODO: DOC keep max values on arrays and single field.
            ROS_DEBUG_STREAM(" --> emo: node (pre) (" << node.emotion << ", " << node.value << ")");
            ROS_DEBUG_STREAM(" -->      child      (" << child.emotion << ", " << child.value << ")");
            if (is_leaf) {
                // update just one emotion
                if (child.value > node.children_values[child.emotion]) {
                    node.children_values[child.emotion] = child.value;
                }
            } else {
                // update all emotions
                std::vector<int32_t>::const_iterator e_it = child.children_emotions.begin();
                std::vector<float>::const_iterator f_it = child.children_values.begin();
                for (; e_it != child.children_emotions.end() && f_it != child.children_values.end(); ++e_it, ++f_it) {
                    if (*f_it > node.children_values[*e_it]) {
                        // keep max on list
                        node.children_values[*e_it] = *f_it;
                    }
                }

            }

            // update max emotion
            if (child.value > node.value) {
                node.emotion = child.emotion;
                node.value = child.value;
            }
            ROS_DEBUG_STREAM(" -->      node (post)(" << node.emotion << ", " << node.value << ")");
            return true;
        }

    }
}

