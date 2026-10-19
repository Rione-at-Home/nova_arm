# One frame from one FlexiTac pad (16 x 32), with the derived contact features and link health.
# Field names match flexigrasp_msgs/TactileFrame so the FlexiGrasp viewer and bag tools carry over.

std_msgs/Header header        # stamp = host time when the bytes were read; frame_id = <side>_gripper_tactile
string finger                 # arm side: "right" or "left"
uint32 seq                    # frames received since the node started (gaps = frames lost upstream of ROS)
uint16 rows                   # 16
uint16 cols                   # 32
uint8[] raw                   # rows*cols raw 8-bit ADC values, row-major
bool calibrated               # false while the baseline is being collected; delta/features are zero then
float32[] delta               # rows*cols, raw - baseline, masked / out-of-region taxels set to 0
float32 peak                  # max(delta)
float32 total                 # sum(max(delta - threshold, 0)), the contact signal the grasp controller uses
int32 contact_taxels          # taxels with delta > threshold
bool in_contact               # contact_taxels >= contact_min_taxels
float32 centroid_row          # pressure-weighted centroid, NaN with no contact
float32 centroid_col
uint32 bad_frames             # frames rejected by the parser since the port was opened
uint32 dropped_bytes          # bytes skipped while resynchronising
float32 rate_hz               # frames received in the last second
