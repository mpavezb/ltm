#include <ltm/db/episode_metadata.h>


namespace ltm {
    namespace db {

        void EpisodeMetadataBuilder::setup(EpisodeCollectionPtr coll) {
            this->_coll = coll;
        }

        void EpisodeMetadataBuilder::make_meta_episode(const Episode &node, MetadataPtr meta) {
            meta->append("uid", (int) node.uid);
            meta->append("type", node.type);
            meta->append("parent_id", (int) node.parent_id);
            meta->append("children_ids", node.children_ids);
            meta->append("tags", node.tags);
            meta->append("children_tags", node.children_tags);
        }

        void EpisodeMetadataBuilder::make_meta_info(const Info &node, MetadataPtr meta) {
            MetadataPtr info = create_metadata();
            info->append("source", node.source);
            meta->appendMeta("info", info);
        }

        void EpisodeMetadataBuilder::make_meta_when(const When &node, MetadataPtr meta) {
            double start = node.start.sec + node.start.nsec * pow10(-9);
            double end = node.end.sec + node.end.nsec * pow10(-9);
            MetadataPtr when = create_metadata();
            when->append("start", start);
            when->append("end", end);
            meta->appendMeta("when", when);
        }

        void EpisodeMetadataBuilder::make_meta_where(const Where &node, MetadataPtr meta) {
            MetadataPtr where = create_metadata();
            where->append("frame_id", node.frame_id);
            where->append("map_name", node.map_name);
            where->append("position_x", node.position.x);
            where->append("position_y", node.position.y);
            where->append("location", node.location);
            where->append("area", node.area);
            where->append("children_locations", node.children_locations);
            where->append("children_areas", node.children_areas);
            meta->appendMeta("where", where);
        }

        void EpisodeMetadataBuilder::make_meta_what(const What &node, MetadataPtr meta) {
            MetadataPtr what = create_metadata();

            // STREAMS
            std::vector<Metadata::ConstPtr> stream_v;
            std::vector<ltm::StreamRegister>::const_iterator s_it;
            for (s_it = node.streams.begin(); s_it != node.streams.end(); ++s_it) {
                MetadataPtr entry = create_metadata();
                entry->append("type", s_it->type);
                entry->append("uid", (int) s_it->uid);
                stream_v.push_back(entry);
            }
            what->appendMeta("streams", stream_v);

            // ENTITIES
            std::vector<Metadata::ConstPtr> entity_v;
            std::vector<ltm::EntityRegister>::const_iterator e_it;
            for (e_it = node.entities.begin(); e_it != node.entities.end(); ++e_it) {
                MetadataPtr entry = create_metadata();
                entry->append("type", e_it->type);
                entry->append("uid", (int) e_it->uid);
                entity_v.push_back(entry);
            }
            what->appendMeta("entities", entity_v);

            meta->appendMeta("what", what);
        }

        void EpisodeMetadataBuilder::make_meta_relevance(const Relevance &node, MetadataPtr meta) {
            MetadataPtr relevance = create_metadata();
            make_meta_relevance_emotional(node.emotional, relevance);
            make_meta_relevance_historical(node.historical, relevance);
            meta->appendMeta("relevance", relevance);
        }

        void EpisodeMetadataBuilder::make_meta_relevance_historical(const HistoricalRelevance &node, MetadataPtr meta) {
            MetadataPtr historical = create_metadata();
            historical->append("value", node.value);

            char buff[16];
            snprintf(buff, sizeof(buff), "%04u/%02u/%02u", node.last_update.year, node.last_update.month,
                     node.last_update.day);
            std::string last_update = buff;
            snprintf(buff, sizeof(buff), "%04u/%02u/%02u", node.next_update.year, node.next_update.month,
                     node.next_update.day);
            std::string next_update = buff;
            historical->append("last_update", last_update);
            historical->append("next_update", next_update);
            meta->appendMeta("historical", historical);
        }

        void EpisodeMetadataBuilder::make_meta_relevance_emotional(const EmotionalRelevance &node, MetadataPtr meta) {
            MetadataPtr emotional = create_metadata();
            emotional->append("emotion", node.emotion);
            emotional->append("value", node.value);
            emotional->append("children_emotions", node.children_emotions);
            emotional->append("children_values", node.children_values);
            meta->appendMeta("emotional", emotional);
        }

        MetadataPtr EpisodeMetadataBuilder::make_metadata(const Episode &episode) {
            MetadataPtr meta = create_metadata(true);
            make_meta_episode(episode, meta);
            make_meta_info(episode.info, meta);
            make_meta_when(episode.when, meta);
            make_meta_where(episode.where, meta);
            make_meta_what(episode.what, meta);
            make_meta_relevance(episode.relevance, meta);
            return meta;
        }

        MetadataPtr EpisodeMetadataBuilder::create_metadata(bool root) {
            if (root)
                return _coll->createMetadata();
            return _coll->createNestedMetadata();
        }
        
    }
}