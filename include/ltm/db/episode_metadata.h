#ifndef LTM_DB_EPISODE_METADATA_H
#define LTM_DB_EPISODE_METADATA_H

#include <ltm/db/types.h>
#include <ltm/Episode.h>

namespace ltm {
    namespace db {

        typedef ltm_db::MessageCollection<ltm::Episode> EpisodeCollection;
        typedef boost::shared_ptr<EpisodeCollection> EpisodeCollectionPtr;

        class EpisodeMetadataBuilder {
        private:
            void make_meta_episode(const Episode &node, MetadataPtr meta);
            void make_meta_info(const Info &node, MetadataPtr meta);
            void make_meta_when(const When &node, MetadataPtr meta);
            void make_meta_where(const Where &node, MetadataPtr meta);
            void make_meta_what(const What &node, MetadataPtr meta);
            void make_meta_relevance(const Relevance &node, MetadataPtr meta);
            void make_meta_relevance_historical(const HistoricalRelevance &node, MetadataPtr meta);
            void make_meta_relevance_emotional(const EmotionalRelevance &node, MetadataPtr meta);

        public:
            MetadataPtr make_metadata(const Episode &episode, EpisodeCollectionPtr &coll);
        };
    }
}

#endif //LTM_DB_EPISODE_METADATA_H
