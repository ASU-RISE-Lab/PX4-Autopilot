
uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the data sample on which this message is based (microseconds)

float32[3] tau          # torque setpoint about NED frame (body frame) (in N.m)

float32[3] f          # thrust setpoint along NED frame (body frame) (in N)
