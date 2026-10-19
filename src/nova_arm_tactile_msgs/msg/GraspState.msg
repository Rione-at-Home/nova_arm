# Published by grasp_node on /arm/<side>/grasp_state at a fixed rate, also while no goal is active,
# so a task node can see a hold become a slip or a drop.

uint8 IDLE=0          # not commanding the gripper
uint8 OPENING=1
uint8 CLOSING=2       # moving toward closed, waiting for tactile contact
uint8 SQUEEZING=3     # in contact, tightening toward the target total
uint8 HOLDING=4       # target reached; tightens again on slip, up to max_squeeze_rad past contact
uint8 LOST=5          # was holding, contact disappeared (object dropped or pulled out)
uint8 FAULT=6         # stopped: sensor stale, arm absent, motion inhibited, stall without contact

std_msgs/Header header
string side
uint8 state
string state_name
string detail                 # why the last transition happened (e.g. fault reason)
bool tactile_ok               # pad connected, calibrated and fresh
bool in_contact
float32 total
int32 contact_taxels
float32 centroid_row
float32 centroid_col
float32 gripper_command       # rad, last goal sent on /arm_command (NaN if none yet)
float32 gripper_position      # rad, measured from /joint_states (NaN if never read)
float32 contact_position      # rad, gripper command at first contact (NaN if none)
float32 target_total
