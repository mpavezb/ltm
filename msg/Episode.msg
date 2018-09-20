# Identification
uint32 uid
uint8 type
string[] tags
ltm/Info info

# Tree Information
uint32 parent_id
uint32[] children_ids
string[] children_tags

# Episode  Information
ltm/What  what
ltm/When  when
ltm/Where where

# Relevance
ltm/Relevance relevance

# Available types
uint8 LEAF=0
uint8 EPISODE=1