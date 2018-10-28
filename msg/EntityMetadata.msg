uint32 uid     # entity uid in database

# log uids
uint32 log_uid # entity register uid at recall time
uint32 init_log # first entity register uid in database
uint32 last_log # last entity register uid in database

# log stamps
time stamp     # recall time (log stamp)
time init_stamp # first registered stamp
time last_stamp # last registered stamp
