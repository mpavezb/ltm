#ifndef LTM_DB_TYPES_H
#define LTM_DB_TYPES_H

#include <ltm_db/interface/message_with_metadata.h>
#include <ltm_db/mongo/database_connection.h>

typedef ltm_db_mongo::Query Query;
typedef ltm_db_mongo::Query::Ptr QueryPtr;

typedef ltm_db::Metadata Metadata;
typedef ltm_db::Metadata::Ptr MetadataPtr;

typedef boost::shared_ptr<ltm_db_mongo::MongoDatabaseConnection> DBConnectionPtr;


#endif //LTM_DB_TYPES_H
