#python
import math

# Updated PID gains for balancing
kp_balance = 40.0
ki_balance = 4.0
kd_balance = 20.0

# Updated LQR gains for yaw control
k1_yaw = 10.0
k2_yaw = 10.0

# Desired yaw angle in radians
yaw_setpoint = 0
drive_speed = 0

# Initialize variables
prev_error_balance = 0.0
integral_balance = 0.0
prev_error_yaw = 0.0
integral_yaw = 0.0

def sysCall_init():
    global front_motor_handle, reference, bike, drive_motor
    # Get object handles
    front_motor_handle = sim.getObject('/reference_frame/front_motor')
    reference = sim.getObject('/reference_frame')
    bike = sim.getObject('/reference_frame/bike_respondable')
    drive_motor = sim.getObject('/reference_frame/drive_motor')

def sysCall_actuation():
    #balance Control
    orientation = sim.getObjectOrientation(reference, bike)
    roll = orientation[1]
    error_balance = 0.000 - roll
    global prev_error_balance, integral_balance
    integral_balance += error_balance
    derivative_balance = error_balance - prev_error_balance
    balance_control_signal = kp_balance * error_balance + ki_balance * integral_balance + kd_balance * derivative_balance
    sim.setJointTargetVelocity(front_motor_handle, balance_control_signal)

    # Yaw Control
    roll, pitch, yaw = sim.getObjectOrientation(bike, reference)
    yaw_error = yaw_setpoint - yaw
    yaw_control = calculate_control_velocity(yaw_error)
    
    # Apply control signals for balancing and yaw control
    sim.setJointTargetVelocity(front_motor_handle, balance_control_signal + yaw_control)

    # Drive Speed Control
    sim.setJointTargetVelocity(drive_motor, drive_speed)

    # Update previous errors for the next iteration
    prev_error_balance = error_balance

def sysCall_sensing():
    global yaw_setpoint, drive_speed
    roll, pitch, yaw = sim.getObjectOrientation(bike, reference)
    message, data, data2 = sim.getSimulatorMessage()
    
    if message == sim.message_keypress:
        if data[0] == 2007:  # forward up arrow
            drive_speed = -20  # add drive wheel speed here
        elif data[0] == 2008:  # backward down arrow
            drive_speed = 20  # add drive wheel speed here
        elif data[0] == 2009:  # left arrow key
            yaw_setpoint = -0.1  # change yaw_setpoint for required turning
        elif data[0] == 2010:  # right arrow key
            yaw_setpoint = 0.1  # change yaw_setpoint for required turning
    else:
        drive_speed = 0
        yaw_setpoint = yaw

def sysCall_cleanup():
    pass

def calculate_control_velocity(yaw_error):
    _, angular_velocity = sim.getObjectVelocity(bike)
    velocity = -k1_yaw * yaw_error - k2_yaw * angular_velocity[2]
    return velocity
