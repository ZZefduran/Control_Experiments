
from ros_msg.MotionCommand.msg import MotionCommand

### create cmd with type dict

motor_id = 100
pos = 0
vel = 0
torque = 0
kp = 100
kd = 5

request_id = 0
cmd = MotionCommand()
cmd.header.frame_id = str(request_id)
cmd.drive_ids.append(motor_id)
cmd.target_position.append(float(pos))
cmd.target_velocity.append(float(vel))
cmd.target_torque.append(float(torque))
cmd.kp.append(float(kp))
cmd.kd.append(float(kd))
publisher.publish(cmd)

request_id += 1


### create ros publisher

## publish cm