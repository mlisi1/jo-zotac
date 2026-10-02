std_msgs/Header header
uint32 track_id
uint32 track_age          # frames this track has been alive

uint8 STATUS_STATIC              = 0  # confirmed track, no motion evidence
uint8 STATUS_DYNAMIC             = 1  # classified dynamic by classificationCB
uint8 STATUS_POTENTIALLY_DYNAMIC = 2  # YOLO-identified but currently stationary
uint8 STATUS_TRACKED             = 3  # KF speed >= dynamic_velocity_threshold, not yet classified
uint8 status

geometry_msgs/Pose pose
geometry_msgs/Vector3 size
geometry_msgs/Twist twist
geometry_msgs/Accel accel
