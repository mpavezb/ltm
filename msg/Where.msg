# numerical information
string frame_id                # related to pose
string map_name                # where the pose was recorded
geometry_msgs/Point position   # 2D position in map

# semantic location information
string location               # e.g. Living Room
string area                   # e.g. Arena 1

# children information
geometry_msgs/Point[] children_hull # Convex Hull for parents
                                    # only valid if all children
                                    # share frame_id and map_name
string[] children_locations
string[] children_areas
