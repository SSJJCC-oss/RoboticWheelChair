
# This code achieves the control of the  wheel chair with the aid of a large language model, OpenAI API
# The wheel chair user speaks into the microphone indicting the intent. example : "Where can I find the milk in the house". 
# This speech is processed by OPen API to makeout the user's intent.
# The response given by OpenAPI would contain the 'key word' which will  indicate the location in the house where the wheel chair 
# is to be taken .example, "Kitchen", "Bathroom" etc..
# The shortest path to the destination is derived using A star algorithm. The floor of the house is divided into
# grids which divides the floor into calls. The chequered floor in webots indicates each cell. The cells are numbered from left-top corner
# starting as ( 0,0), (0,1)....(0,9) in the horizontal direction and (0,0), (1,0)......(9,0) in the vertical downwards direction.
# The current  position of the wheel chair in the house is obtained from the GPS device built into the wheel chair. An IMU device is 
# also intergrated into the wheel chair to determine the direction, by calculating the yaw angle. 


#Required library funcitons are imported.
import math
import heapq
import time
import speech_recognition as sr
import openai
from controller import Robot

# Constants
VELOCITY = 5.0 # the velocity set for the wheels
DISTANCE_BETWEEN_WHEELS = 4.5
GRID_WIDTH = 10 # Grid formed based on the floor dimensions, which is 10 meters X 10 meters
GRID_HEIGHT = 10
DIRECTIONS = ['N', 'E', 'S', 'W']  # North, East, South, West
MOVE_DURATION = 2700  # Duration to move forward (in milliseconds)
CELL_SIZE = 1.0  # Size of each cell in meters

# The API kepy to be replaced with a valid API key
openai.api_key = 'sk-Ut11SuXd9DBcfvBhjOEsVpzJQAvXPoMsoBwNM8f2LWT3BlbkFJjebOoleJA6QRNoYVG13XSQSD3sMee0SXvP-kkfzZ8A'  


# The grid coordinates of each room is defined by a specific cell in that room.
VOICE_TO_POSITION = {
    'bedroom': (1, 0),
    'living room': (7, 4),
    'kitchen': (1, 9),
    'bathroom': (8, 6)
}

# Initiate the Robot controller class.
class RobotController:
    def __init__(self):
        # Initialize the robot and devices
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Initialize motors
        self.leftfront_motor = self.robot.getDevice('lfmotor')
        self.leftback_motor = self.robot.getDevice('lbmotor')
        self.rightfront_motor = self.robot.getDevice('rfmotor')
        self.rightback_motor = self.robot.getDevice('rbmotor')

        self.leftfront_motor.setPosition(float('inf'))
        self.leftback_motor.setPosition(float('inf'))
        self.rightfront_motor.setPosition(float('inf'))
        self.rightback_motor.setPosition(float('inf'))

        # Initialize sensors
        self.imu = self.robot.getDevice('imu')  # Initialize IMU
        self.imu.enable(self.timestep)
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)

        # Initialize voice recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Ensure motors are stopped at the start
        self.stop_motors()
        self.path = []
        self.visited_positions = []


        # Convert GPS coordinates to matrix coordinates
    def gps_to_matrix(self, gps_x, gps_y):
        
        # Define the GPS boundaries corresponding to the grid dimensions
        gps_x_min, gps_x_max = -5, 5
        gps_y_min, gps_y_max = 5, -5

        # Define grid cell size based on GPS range and grid size
        grid_x_size = (gps_x_max - gps_x_min) / GRID_WIDTH
        grid_y_size = (gps_y_max - gps_y_min) / GRID_HEIGHT

        # Convert GPS coordinates to grid coordinates
        grid_x = int((gps_x - gps_x_min) / grid_x_size)
        grid_y = int((gps_y - gps_y_min) / grid_y_size)

        # Handle the special cases for x and y near the edges
        if -5 <= gps_x < -4:
            grid_x = 0
        if 4 <= gps_y <= 5:
            grid_y = 0

        # Ensure grid coordinates are within bounds
        grid_x = max(0, min(GRID_WIDTH - 1, grid_x))
        grid_y = max(0, min(GRID_HEIGHT - 1, grid_y))

        # Return grid coordinates (swap x and y for correct mapping)
        return grid_y, grid_x

    #Get the current position from the GPS and map it to grid coordinates
    def get_current_position(self):
        
        while True:
            position = self.gps.getValues()
            x, y, z = position[0], position[1], position[2]
            print(f"Raw GPS values: x={x}, y={y}, z={z}")  # Added print statement

            # Check if the GPS values are valid (not NaN)
            if not math.isnan(x) and not math.isnan(y):
                # Apply conversion
                grid_x, grid_y = self.gps_to_matrix(x, y)
                print(f"Mapped Grid Position: grid_x={grid_x}, grid_y={grid_y}")  # Added print statement
                return (grid_x, grid_y)
            else:
                print("Waiting for valid GPS data...")
                self.robot.step(self.timestep)  # Continue stepping until valid GPS data is available
    # Preprocess the recognized command and map to a position.
    def preprocess_and_recognize(command):
        
        command = command.lower()
        for key, position in VOICE_TO_POSITION.items():
            if key in command:
                print(f"Command matched: {key}, Position: {position}")
                return position
        print(f"Command not recognized: {command}")
        return None

    #Defining  heuristic values for A star algorithm
    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    #Set velocity for all four motors.
    def set_velocity(self, velocity_left, velocity_right):
        
        self.leftfront_motor.setVelocity(velocity_left)
        self.leftback_motor.setVelocity(velocity_left)
        self.rightfront_motor.setVelocity(velocity_right)
        self.rightback_motor.setVelocity(velocity_right)

        #Stop all motors.
    def stop_motors(self):
   
        self.set_velocity(0.0, 0.0)

        #Move the wheelchair forward for a specified duration
    def move_forward(self):
        
        duration = MOVE_DURATION / 1000.0  # Convert milliseconds to seconds
        start_time = self.robot.getTime()
        print(f"Moving forward")
        self.set_velocity(VELOCITY, VELOCITY)
        while self.robot.step(self.timestep) != -1:
            if self.robot.getTime() - start_time > duration:
                break
        self.stop_motors()
        print("Finished moving forward.")

        # Get the wheelchair's current heading using the Inertial Measurement Unit (IMU)
    def get_heading(self):
        
        imu_rotation = self.imu.getRollPitchYaw()[2]  # Get yaw (heading)
        heading = (imu_rotation * 180 / math.pi) % 360  # Convert to degrees and normalize

        if 45 <= heading < 135:
            return 'W'
        elif 135 <= heading < 225:
            return 'S'
        elif 225 <= heading < 315:
            return 'E'
        else:
            return 'N'

        #Turn the robot to face the target direction
    def turn_to_direction(self, target_direction):
        
        current_direction = self.get_heading()
        print(f"Turning from {current_direction} to {target_direction}...")

        if current_direction == target_direction:
            print("Already facing the target direction.")
            return

        # Calculate turns needed
        current_index = DIRECTIONS.index(current_direction)
        target_index = DIRECTIONS.index(target_direction)
        turns_needed = (target_index - current_index) % 4

        
        
        turn_durationR = 5.41
        turn_durationL = 5.607
        turn_duration180 = 10.82



        # Duration for each type of turn
        turn_durations = {
            1: turn_durationR,  # 90 degrees to the right
            3: turn_durationL,  # 90 degrees to the left
            2: turn_duration180  # 180 degrees
        }

        if turns_needed in turn_durations:
            if turns_needed == 3:  # 90 degrees to the right
                self.set_velocity(-VELOCITY/4, VELOCITY/4)
            elif turns_needed == 1:  # 90 degrees to the left
                self.set_velocity(VELOCITY/4, -VELOCITY/4)
            elif turns_needed == 2:  # 180 degrees
                self.set_velocity(-VELOCITY/4, VELOCITY/4)

            turn_duration = turn_durations[turns_needed]

            # Execute turn
            start_time = self.robot.getTime()
            while self.robot.step(self.timestep) != -1:
                if self.robot.getTime() - start_time > turn_duration:
                    break

            # Fine-tuning the turn
            print("Fine-tuning...")
            fine_tune_start_time = self.robot.getTime()
            correction_duration = 0.5  # Shortened duration for correction

            while self.robot.step(self.timestep) != -1:
                if self.robot.getTime() - fine_tune_start_time > correction_duration:
                    break

                final_heading = self.get_heading()
                heading_error = (DIRECTIONS.index(target_direction) - DIRECTIONS.index(final_heading)) % 4

                if heading_error == 3:  # Small right adjustment
                    self.set_velocity(VELOCITY * 0.1, -VELOCITY * 0.1)
                elif heading_error == 1:  # Small left adjustment
                    self.set_velocity(-VELOCITY * 0.1, VELOCITY * 0.1)
                else:
                    break

            # Stop the motors after adjustments
            self.stop_motors()
            print("Finished turning.")

        #Turn the robot to face the target direction and then move forward.
    def move(self, target_direction):
        
        current_direction = self.get_heading()

        # Turn to the target direction
        if current_direction != target_direction:
            self.turn_to_direction(target_direction)

        # Move forward
        self.move_forward()


        #Get coordinates from voice input.
    def get_coordinates_from_voice(self):
        
        recognizer = sr.Recognizer()
        with sr.Microphone() as source:
            print("Listening for voice command...")
            audio = recognizer.listen(source)

        try:
            command = recognizer.recognize_google(audio).lower()
            print(f"Recognized command: {command}")
            return command

        except sr.UnknownValueError:
            print("Sorry, I did not understand the audio.")
            return None
        except sr.RequestError:
            print("Sorry, there was an error with the speech recognition service.")
            return None
            
            
    def interpret_command_with_llm(self, command):
        if command:
            try:
                # ChatCompletion endpoint is used for interacting with gpt-3.5-turbo
                response = openai.ChatCompletion.create(
                    model="gpt-3.5-turbo", 
                    messages=[
                        {"role": "system", "content": "You are a helpful assistant that understands and interprets user commands for a wheelchair in a house environment."},
                        {"role": "user", "content": f"The user said: '{command}'. Determine the user's intent and provide a corresponding action for a wheelchair. For example, if the user asks for 'milk,' assume the wheel chair should take the user to the kitchen."}
                    ],
                    max_tokens=1000,
                    temperature=0.5
                )
                # Retreiving the response of openAI
                action = response['choices'][0]['message']['content'].strip()
                print(f"Interpreted Action: {action}")
                # Use the VOICE_TO_POSITION dictionary to map the command to coordinates
                for key, position in VOICE_TO_POSITION.items():
                    if key in action:
                        print(f"Command matched: {key}, Position: {position}")
                        return position
            except Exception as e:
                print(f"Error with OpenAI API: {e}")
                return None
        else:
            print("No command provided to interpret.")
            return None

        # Executing the path using A* algorithm.
    def find_and_execute_path(self, board, start_node, goal_node):
        
        open_list = [(0 + heuristic(start_node, goal_node), 0, start_node, "")]
        heapq.heapify(open_list)
        closed_list = set()

        while open_list:
            if self.robot.step(self.timestep) == -1:
                break

            _, cost, current_node, path = heapq.heappop(open_list)
            if current_node in closed_list:
                continue
            closed_list.add(current_node)
            if current_node == goal_node:
                print("Path found:", path)
                break
            x, y = current_node
            neighbors = [
                ((x - 1, y), 'U'),
                ((x + 1, y), 'D'),
                ((x, y - 1), 'L'),
                ((x, y + 1), 'R')
            ]
            for neighbor, direction in neighbors:
                nx, ny = neighbor
                if 0 <= nx < len(board) and 0 <= ny < len(board[0]) and board[nx][ny] != 1:
                    new_cost = cost + 1
                    estimated_cost = new_cost + heuristic(neighbor, goal_node)
                    heapq.heappush(open_list, (estimated_cost, new_cost, neighbor, path + direction))

        for direction in path:
            if direction == 'U':
                self.move('N')
            elif direction == 'D':
                self.move('S')
            elif direction == 'L':
                self.move('W')
            elif direction == 'R':
                self.move('E')

# Heuristic function for A* algorithm.
def heuristic(a, b):
    
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def main():
    controller = RobotController()
     
    while controller.robot.step(controller.timestep) != -1:
        # Listening to the voice command and determining the goal node in each loop.
        # The below Matrix denotes each cell of the house floor, and the '0' and '1' indicates whether there is an obstacle in the cell.
        voice_command = controller.get_coordinates_from_voice()
        goal_node = controller.interpret_command_with_llm(voice_command)
        if goal_node:
            start_node = controller.get_current_position()  # Example starting point
            board = [
                [1, 1, 1, 0, 0, 0, 1, 1, 1, 1],
                [0, 1, 1, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 1, 1, 0, 0],
                [1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [1, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                [1, 0, 0, 0, 0, 1, 0, 1, 1, 1],
                [1, 1, 1, 0, 1, 1, 0, 0, 0, 1],
                [1, 1, 1, 0, 1, 1, 0, 0, 0, 1]
            ]
            controller.find_and_execute_path(board, start_node, goal_node)


if __name__ == "__main__":
    main()
