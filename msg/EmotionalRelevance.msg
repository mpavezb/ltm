# Information provided by the emotion engine.
string software               # e.g. "fake"
string software_version       # e.g. "0.0.0"

# Registered Emotions
# Do not impose names, quantity or bounds for values. However, it is
# adviced to align the emotion names to the ones provided below
# and to keep values in the range [0, 1].
string[] registered_emotions  # e.g. ["happy","surprised","angry","sad", ...]
float32[] registered_values   # e.g. [0.6, 0.2, 0.1, 0.1, ...]

# Registered emotion and value for the episode.
# - emotion is specified by the integer associated to one emotion below.
# - value is a floating number between 0 (minimum) and 1 (max emotion strength).
int32 emotion  # e.g. 3 (SURPRISE)
float32 value  # e.g. 0.5

# Available emotions (Plutchik's Wheel of emotions).
uint8 JOY=0
uint8 TRUST=1
uint8 FEAR=2
uint8 SURPRISE=3
uint8 SADNESS=4
uint8 DISGUST=5
uint8 ANGER=6
uint8 ANTICIPATION=7