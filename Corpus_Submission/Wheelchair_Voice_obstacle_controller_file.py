# The below code controls a wheel chair robot to move forward, backwards , and turn left or right based on voice commands
# Also the robot detects obstacles in front and back, through distance sensors (ultrasonic sensors) mounted on the wheel chair.
# Distance sensors are also mounted on the sides of the wheel chair, they measure the distance to the obstacles on the sides, 
#like a wall ot something. When the front sensors detect and obstacle within 0.5 meters, If the left sensor value is more than the 
# right value, the wheel chair turns left and if right distance is sensed more, it turns towards right.


# the below library functions are imported for realising the control logic.
import speech_recognition as sr
import threading
from controller import Robot

# Initialize recognizer class (for recognizing speech)
recognizer = sr.Recognizer()

# Initialize Webots Robot instance
robot = Robot()
TIMESTEP = int(robot.getBasicTimeStep())

# Initialize the motors of the 4 wheels of the wheel chair.
# The wheel chair has been equipped with 4 motors, named as left front, left back, right front, right back.
def initialize_motors(robot):
    motors = {
        'left_front': robot.getDevice('lfmotor'),
        'left_back': robot.getDevice('lbmotor'),
        'right_front': robot.getDevice('rfmotor'),
        'right_back': robot.getDevice('rbmotor')
    }
    for motor in motors.values():
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)  # motors shall be standstill initially.
    return motors

# The below code captures the voice command, adjust for ambient noise, and records the audio bit using the microphone.
def capture_voice():
    with sr.Microphone() as source:
        print("Adjusting for ambient noise... Please wait.")
        recognizer.adjust_for_ambient_noise(source, duration=1)
        print("Listening...")
        audio = recognizer.listen(source)
        print("Recording complete.")
        return audio

# The below function preprocesses the recorded audio bit,and convert into lower case text using Google API
# and prints error messages if the audio could not be recognised, or if the conenction to Google API is not eatablished.
def preprocess_and_recognize(audio):
    try:
        print("Recognizing speech...")
        command = recognizer.recognize_google(audio)
        print(f"Command: {command}")
        return command.lower() 
    except sr.UnknownValueError:
        print("Sorry, I did not understand that.")
        return None
    except sr.RequestError:
        print("Sorry, my speech service is down.")
        return None

# The wheel control function is defined as below, the motors rotate forward or reverse, at the below defined speed.
def control_robot(command, motors):
    left_speed = 3.0  # both left motors are assigned this speed.
    right_speed = 3.0  # both right motors are assigned this speed.



    # six distance sensors are mounted on the wneel chair as below
    #ds front sensor, looking straight ahead,  ds front right, ds front left looking forward at an angle.
    # ds lef and right measures distance to the sides and ds back mesures distance to obstacles in the backward direction.
    ds_front = robot.getDevice('ps1')
    ds_frontright = robot.getDevice('ps6')
    ds_frontleft = robot.getDevice('ps12')
    ds_left = robot.getDevice('ps3')
    ds_right = robot.getDevice('ps4')
    ds_back = robot.getDevice('ps2')
    ds_front.enable(10)
    ds_frontright.enable(10)
    ds_frontleft.enable(10)
    ds_left.enable(10)
    ds_right.enable(10)
    ds_back.enable(10)
    
    front_value = ds_front.getValue()
    frontright_value=ds_frontright.getValue()
    frontleft_value=ds_frontleft.getValue()
    left_value = ds_left.getValue()
    right_value = ds_right.getValue()
    back_value = ds_back.getValue()


    # Based on the recognised voice command, like 'forward', 'backward', 'left', 'right' etc, the wheels rotation is set as below.
    if command:
            if "forward" in command:
                if front_value < 0.5 or frontright_value <0.5 or frontleft_value<0.5: # the three front distance sensors measures less than 0.5 meters
                    if left_value < right_value: # the right  distance sensor value is more than the left distance sensor, indicating there is more space towards right.
                          # the wheel chair turns right
                          motors['left_front'].setVelocity(left_speed/4)
                          motors['left_back'].setVelocity(left_speed/4)
                          motors['right_front'].setVelocity(-right_speed/4)
                          motors['right_back'].setVelocity(-right_speed/4)
                          # a turn duration of 5 secs is set, so that the wheel chair will not get stuck at a tigh corner, where the left and right ds values might read equal.
                          start_time = robot.getTime()
                          duration = 5
                          while True:
                              robot.step(TIMESTEP)
                              current_time = robot.getTime()
                              if current_time - start_time >= duration:
                                 break
                         
                    else: 
                            motors['left_front'].setVelocity(-left_speed/4) # if left distance sensor reads more than the right one,the wheelchair will turn left for 5 secs.
                            motors['left_back'].setVelocity(-left_speed/4)
                            motors['right_front'].setVelocity(right_speed/4)
                            motors['right_back'].setVelocity(right_speed/4)
                            start_time = robot.getTime()
                            duration = 5
                            while True:
                              robot.step(TIMESTEP)
                              current_time = robot.getTime()
                              if current_time - start_time >= duration:
                                 break
                                 
                else:
                          motors['left_front'].setVelocity(left_speed) # If the front sensors does not detect obstacle within o.5 meters, the wheels are commanded to go straight.
                          motors['left_back'].setVelocity(left_speed)
                          motors['right_front'].setVelocity(right_speed)
                          motors['right_back'].setVelocity(right_speed)
                     
                                     
                    
            elif "slow" in command: # This 'slow' function is added for enabling forward movement at a very small speed. usefule while approaching a partially opened door or narrow spaces.
                motors['left_front'].setVelocity(left_speed/10)
                motors['left_back'].setVelocity(left_speed/10)
                motors['right_front'].setVelocity(right_speed/10)
                motors['right_back'].setVelocity(right_speed/10)
                
            elif "backward" in command: # for moving backward.
                  if back_value < 0.5: # if an obtascle is detected behind while moving backwards, the wheels are commanded to stop.
                     motors['left_front'].setVelocity(0.0)
                     motors['left_back'].setVelocity(0.0)
                     motors['right_front'].setVelocity(0.0)
                     motors['right_back'].setVelocity(0.0)
            
                  else:
                     motors['left_front'].setVelocity(-left_speed/5) # if no obstacle is detected behind, moves backwards.
                     motors['left_back'].setVelocity(-left_speed/5)
                     motors['right_front'].setVelocity(-right_speed/5)
                     motors['right_back'].setVelocity(-right_speed/5)
                
                
            elif "right" in command: # If voice command is 'right', wheel chair turns right.
                motors['left_front'].setVelocity(left_speed/8)
                motors['left_back'].setVelocity(left_speed/8)
                motors['right_front'].setVelocity(-right_speed/8)
                motors['right_back'].setVelocity(-right_speed/8)
            elif "left" in command: # If voice command is 'left', wheel chair turns left.
                motors['left_front'].setVelocity(-left_speed/8)
                motors['left_back'].setVelocity(-left_speed/8)
                motors['right_front'].setVelocity(right_speed/8)
                motors['right_back'].setVelocity(right_speed/8)
            elif "stop" in command: # If voice command is 'stop', wheel chair stops.
                motors['left_front'].setVelocity(0.0)
                motors['left_back'].setVelocity(0.0)
                motors['right_front'].setVelocity(0.0)
                motors['right_back'].setVelocity(0.0)
        

# The below function enables voice recognition and robot control, run  concurrently
def voice_control(motors, shared_command):
    while True:
        audio = capture_voice()
        new_command = preprocess_and_recognize(audio)
        if new_command and new_command != shared_command['current']:
            print(f"Executing command: {new_command}")
            shared_command['current'] = new_command
            control_robot(new_command, motors)

# Main function to run the voice control thread and Webots control loop
def main():
    motors = initialize_motors(robot)
    shared_command = {'current': 'stop'}  # Default command to stop

    # Start voice control in a separate thread
    voice_thread = threading.Thread(target=voice_control, args=(motors, shared_command))
    voice_thread.start()

    # Main control loop for Webots
    while robot.step(TIMESTEP) != -1:
        control_robot(shared_command['current'], motors)

if __name__ == "__main__":
    main()
