std_msgs/Header header

ServoAngle[20] target_angles            # target angle, -120...120° TODO delete this
uint32 time_to_reach_target_angles_ms   # time to reach the target angles, milliSeconds, max speed is 3ms for 1° (0.18s for 60°)
