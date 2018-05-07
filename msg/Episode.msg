# Episode information
uint32 uid
uint8 type
string[] tags
string[] children_tags
ltm/Info info

# Episodic tree information
uint32 parent_id
uint32[] children_ids

# Episodic LTM information
ltm/What  what
ltm/When  when
ltm/Where where

# Relevance
ltm/Relevance relevance

# Available types
uint8 LEAF=0
uint8 EPISODE=1
uint8 CONTEXT=2