#python

# Constants for PID tuning
kp = 75.0  # Proportional gain
ki = 10.1  # Integral gain
kd = 10.01  # Derivative gain

# Variables for PID control
previous_error = 0.0
integral = 0.0

def sysCall_init():
    #Object handles
    global base, motor, pivot, pendulum
    base = sim.getObject('/Base_B')
    motor = sim.getObject('/Elbow_motor_B')
    pivot = sim.getObject('/Pivot_B')
    pendulum = sim.getObject('/Pendulum_B')

def sysCall_actuation():
    #Control Logic Here
    roll,_,_ = sim.getObjectOrientation(pendulum,pivot)
    # PID control calculations
    error = roll  # Replace 0 with the desired setpoint
    global previous_error, integral

    # Calculate PID terms
    proportional = kp * error
    integral += ki * error
    derivative = kd * (error - previous_error)

    # Calculate control signal
    control_signal = proportional + integral + derivative

    # Apply control signal to the motor
    sim.setJointTargetVelocity(motor, control_signal)

    # Update previous error for the next iteration
    previous_error = error
    pass

def sysCall_sensing():
    # Implement any additional sensing or feedback code here
    pass

def sysCall_cleanup():
    # Perform any cleanup if needed
    pass
