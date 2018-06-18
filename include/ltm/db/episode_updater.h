#ifndef LTM_DB_EPISODE_UPDATER_H
#define LTM_DB_EPISODE_UPDATER_H

#include <ltm/db/types.h>
#include <ltm/Episode.h>

namespace ltm {
    namespace db {

        struct EpisodeUpdateHelper {
            std::vector<geometry_msgs::Point> positions;
        };

        class EpisodeUpdater {

        private:
            // init methods
            void update_tree_tags_init(Episode &node);
            void update_tree_info_init(Info &node);
            void update_tree_when_init(When &node);
            void update_tree_where_init(Where &node);
            void update_tree_what_init(What &node);
            void update_tree_relevance_init(Relevance &node);
            void update_tree_relevance_historical_init(HistoricalRelevance &node);
            void update_tree_relevance_emotional_init(EmotionalRelevance &node);

            // add child information methods
            bool update_tree_tags(Episode &node, const Episode &child);
            bool update_tree_info(Info &node, const Info &child);
            bool update_tree_when(When &node, const When &child);
            bool update_tree_where(Where &node, const Where &child, bool is_leaf, int node_uid, int child_uid, std::vector<geometry_msgs::Point> &positions);
            bool update_tree_what(What &node, const What &child);
            bool update_tree_what_streams(What &node, const What &child);
            bool update_tree_what_entities(What &node, const What &child);
            bool update_tree_relevance(Relevance &node, const Relevance &child, bool is_leaf);
            bool update_tree_relevance_historical(HistoricalRelevance &node, const HistoricalRelevance &child);
            bool update_tree_relevance_emotional(EmotionalRelevance &node, const EmotionalRelevance &child, bool is_leaf);

            // ending methods
            bool update_tree_where_last(Where &node, std::vector<geometry_msgs::Point> &positions);

        public:
            void update_tree_init(Episode &node, EpisodeUpdateHelper &helper);
            bool update_tree_last(Episode &node, EpisodeUpdateHelper &helper);
            bool update_from_child(Episode &node, const Episode &child, EpisodeUpdateHelper &helper);
        };
    }
}

#endif //LTM_DB_EPISODE_UPDATER_H
