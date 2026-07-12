from subdyk import get_robot
from pybricks.tools import StopWatch, wait
import umath

from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch



line = ColorSensor(Port.D)

robot = get_robot()
hub = robot.hub
left_motor = robot.left_motor
right_motor = robot.right_motor
stopwatch = robot.stopwatch

async def getBlackLineError():
    # Aseguramos esperar la lectura numérica antes de operar
    valor_reflexion = await line.reflection()
    return (valor_reflexion / 10) - 8

async def avance_gyro(target_heading, final_angle, start_speed, max_speed, end_speed, decel_start_angle, accel_rate, reset_gyro=True, stall_stop=True):
    kp_base = 5
    ki_base = 0.001
    adapt_rate = 0.0001 
    loop_counter = 0
    stopwatch = StopWatch() 
    last_movement_time = stopwatch.time()  
    if reset_gyro:
        hub.imu.reset_heading(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    current_speed = abs(start_speed)
    max_speed = abs(max_speed)
    end_speed = abs(end_speed)
    accel_rate = abs(accel_rate)
    integral_error = 0
    prev_error = 0
    kp_min, kp_max = 0.5 * kp_base, 2 * kp_base
    ki_min, ki_max = 0.5 * ki_base, 2 * ki_base
    while abs(left_motor.angle()) < final_angle:
        left_angle = abs(left_motor.angle()) 
        remaining_distance = final_angle - left_angle 
        if left_angle >= decel_start_angle:
            scale = remaining_distance / (final_angle - decel_start_angle)
            current_speed = end_speed + (max_speed - end_speed) * scale
        else:
            if current_speed < max_speed:
                current_speed += accel_rate
                current_speed = min(current_speed, max_speed)
        error = hub.imu.heading() - target_heading
        integral_error += error  
        kp = kp_base + adapt_rate * abs(error)
        ki = ki_base + adapt_rate * abs(integral_error)
        kp = max(kp_min, min(kp, kp_max))
        ki = max(ki_min, min(ki, ki_max))
        correction = kp * error + ki * integral_error
        loop_counter += 1
        if loop_counter % 100 == 0:
            await wait(10)  

        if stall_stop:
            if abs(left_angle - abs(right_motor.angle())) > 100:
                break
            current_time = stopwatch.time()
            left_sp = abs(left_motor.speed())
            if (current_time - last_movement_time) > 200 and left_sp < 10:
                break
            if left_sp > 10:
                last_movement_time = current_time
        left_motor.dc(current_speed - correction)
        right_motor.dc(current_speed + correction)
    left_motor.hold()
    right_motor.hold()

async def retrocede_gyro(target_heading, final_angle, start_speed, max_speed, end_speed, decel_start_angle, accel_rate, reset_gyro=True, kp_value=7, ki_value=0.001, kd_value=0, stall_stop=True):
    kp_base = kp_value  
    ki_base = ki_value  
    kd_base = kd_value  
    adapt_rate = 0.0001 
    loop_counter = 0  
    stopwatch = StopWatch()  
    last_movement_time = stopwatch.time() 
    if reset_gyro:
        hub.imu.reset_heading(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    current_speed = -abs(start_speed)  
    max_speed = -abs(max_speed)
    end_speed = -abs(end_speed)
    accel_rate = abs(accel_rate)
    integral_error = 0
    prev_error = 0
    kp_min, kp_max = 0.5 * kp_base, 2 * kp_base
    ki_min, ki_max = 0.5 * ki_base, 2 * ki_base
    while abs(left_motor.angle()) < final_angle:
        left_angle = abs(left_motor.angle())  
        remaining_distance = final_angle - left_angle 
        if left_angle >= decel_start_angle:
            scale = remaining_distance / (final_angle - decel_start_angle)
            current_speed = end_speed + (max_speed - end_speed) * scale
        else:
            if current_speed > max_speed:
                current_speed -= accel_rate
                current_speed = max(current_speed, max_speed)
        error = hub.imu.heading() - target_heading
        integral_error += error  
        kp = kp_base + adapt_rate * abs(error)
        ki = ki_base + adapt_rate * abs(integral_error)
        kp = max(kp_min, min(kp, kp_max))
        ki = max(ki_min, min(ki, ki_max))
        correction = kp * error + ki * integral_error + kd_base * (error - prev_error)
        prev_error = error 
        loop_counter += 1
        if loop_counter % 100 == 0:
            await wait(1)  
        if stall_stop:
            if abs(left_angle - abs(right_motor.angle())) > 100:
                break
            current_time = stopwatch.time()
            left_sp = abs(left_motor.speed())
            if (current_time - last_movement_time) > 200 and left_sp < 10:
                break

            if left_sp > 10:
                last_movement_time = current_time
        left_motor.dc(current_speed - correction)
        right_motor.dc(current_speed + correction)
    left_motor.hold()
    right_motor.hold()

async def giro (target_heading, reset_gyro=True, max_speed=80, min_speed=10, stop_threshold=0.05, max_time=1000):
    if reset_gyro:
        hub.imu.reset_heading(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    stopwatch = StopWatch() 
     
    def state():
        theta = hub.imu.heading()  
        theta_dot = hub.imu.angular_velocity()[2] 
        return theta, theta_dot
    
    def lqr_control(current_state, target_heading, K):
        theta, theta_dot = current_state 
        error = target_heading - theta  
        feedback = K[0] * error + K[1] * theta_dot
        return feedback
    K = [8, 0.1]  
    feedforward_gain = 0.3  
    
    while True:
        current_state = state()  
        feedback_input = lqr_control(current_state, target_heading, K)  
        feedforward_input = feedforward_gain * (target_heading - current_state[0]) 
        control_input = feedback_input + feedforward_input  
        if abs(control_input) < min_speed:
            control_input = min_speed if control_input > 0 else -min_speed
        control_input = max(min(control_input, max_speed), -max_speed)  
        left_motor.dc(control_input)
        right_motor.dc(-control_input)
        if stopwatch.time() > max_time:
            break
    left_motor.brake()
    right_motor.brake()

async def giro_pivot (target_heading, active_motor, reset_gyro=True, max_speed=100, min_speed=10, stop_threshold=0.05, max_time=1000):
    if reset_gyro:
        hub.imu.reset_heading(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    stopwatch = StopWatch() 
    
    def state():
        theta = hub.imu.heading()  
        theta_dot = hub.imu.angular_velocity()[2]  
        return theta, theta_dot
    
    def lqr_control(current_state, target_heading, K):
        theta, theta_dot = current_state 
        error = target_heading - theta 
        feedback = K[0] * error + K[1] * theta_dot
        return feedback
    K = [15, 0.5]  
    feedforward_gain = 0.15  
    while True:
        current_state = state() 
        feedback_input = lqr_control(current_state, target_heading, K) 
        feedforward_input = feedforward_gain * (target_heading - current_state[0]) 
        control_input = feedback_input + feedforward_input  
        if abs(control_input) < min_speed:
            control_input = min_speed if control_input > 0 else -min_speed
        control_input = max(min(control_input, max_speed), -max_speed)  
        
        if active_motor == 'left':
            left_motor.dc(control_input)  
            right_motor.hold()  
        elif active_motor == 'right':
            right_motor.dc(-control_input)  
            left_motor.hold() 
        if stopwatch.time() > max_time:
            break
    left_motor.brake()
    right_motor.brake()

async def avance_hasta_linea(target_heading, start_speed=40, max_speed=50, accel_rate=1.2, reset_gyro=True, stall_stop=True):
    kp_base = 5
    ki_base = 0.001
    adapt_rate = 0.0001 
    loop_counter = 0
    stopwatch = StopWatch() 
    last_movement_time = stopwatch.time()  
    if reset_gyro:
        hub.imu.reset_heading(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    current_speed = abs(start_speed)
    max_speed = abs(max_speed)
    accel_rate = abs(accel_rate)
    integral_error = 0
    kp_min, kp_max = 0.5 * kp_base, 2 * kp_base
    ki_min, ki_max = 0.5 * ki_base, 2 * ki_base
    while abs(await getBlackLineError()) >= 8:
        left_angle = abs(left_motor.angle()) 
        if current_speed < max_speed:
            current_speed += accel_rate
            current_speed = min(current_speed, max_speed)
        error = hub.imu.heading() - target_heading
        integral_error += error  
        kp = kp_base + adapt_rate * abs(error)
        ki = ki_base + adapt_rate * abs(integral_error)
        kp = max(kp_min, min(kp, kp_max))
        ki = max(ki_min, min(ki, ki_max))
        correction = kp * error + ki * integral_error
        loop_counter += 1
        if loop_counter % 100 == 0:
            await wait(10)  
        if stall_stop:
            if abs(left_angle - abs(right_motor.angle())) > 100:
                break
            current_time = stopwatch.time()
            left_sp = abs(left_motor.speed())
            if (current_time - last_movement_time) > 200 and left_sp < 10:
                break
            if left_sp > 10:
                last_movement_time = current_time
        left_motor.dc(current_speed - correction)
        right_motor.dc(current_speed + correction)
    left_motor.hold()
    right_motor.hold()



    
