#python
import math

# Updated PID gains for balancing
kp_balance = 40.0
ki_balance = 4.0
kd_balance = 20.0

# Updated LQR gains for yaw control
k1_yaw = 22.7

yaw_setpoint = 0

# Initialize variables
prev_error_balance = 0.0
integral_balance = 0.0

def sysCall_init():
    global front_motor_handle, reference, bike
    # Get object handles
    front_motor_handle = sim.getObject('/reference_frame/front_motor')
    reference = sim.getObject('/reference_frame')
    bike = sim.getObject('/reference_frame/bike_respondable')

def sysCall_actuation():
    global yaw_setpoint
    print("YAW setpoint = ")
    print(yaw_setpoint)
    # Balance Control
    _, roll, _ = sim.getObjectOrientation(reference, bike)
    error_balance = -roll
    global prev_error_balance, integral_balance
    integral_balance += error_balance
    derivative_balance = error_balance - prev_error_balance
    balance_control_signal = kp_balance * error_balance + ki_balance * integral_balance + kd_balance * derivative_balance
    
    
    # Yaw Control
    _, _, yaw = sim.getObjectOrientation(bike, reference)
        
    yaw_error = yaw_setpoint - yaw
    yaw_control = k1_yaw * yaw_error
    
    # Apply control signals for balancing and yaw control
    sim.setJointTargetVelocity(front_motor_handle, balance_control_signal + yaw_control)
    # Update previous errors for the next iteration
    prev_error_balance = error_balance

def sysCall_sensing():
    global yaw_setpoint
    yaw_setpoint = sim.getFloatSignal("yaw_setpoint")
    pass

def sysCall_cleanup():
    pass
