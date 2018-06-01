#ifndef LTM_DB_TYPES_H
#define LTM_DB_TYPES_H

#include <warehouse_ros/message_with_metadata.h>
#include <warehouse_ros_mongo/database_connection.h>

typedef warehouse_ros_mongo::Query Query;
typedef warehouse_ros_mongo::Query::Ptr QueryPtr;

typedef warehouse_ros::Metadata Metadata;
typedef warehouse_ros::Metadata::Ptr MetadataPtr;

typedef boost::shared_ptr<warehouse_ros_mongo::MongoDatabaseConnection> DBConnectionPtr;

#endif //LTM_DB_TYPES_H
