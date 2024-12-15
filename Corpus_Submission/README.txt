Project Title: Language Facilitated task planning robot

A four wheeled robotic wheel chair is realised in a house environment in Webots.
The controller for the robot is made in Python

The following files are in the zipped folder ;

1. Webot world file named 'Wheelchair_worldfile.wbt' 
   This contains the Wheel chair and the house environment. 

2. Controller file named 'Wheelchair_Voice_obstacle_controller_file.py'
   This controller makes the wheelchair robot move based on voice commands. Also it avoids obstacles.
   A brief explanation of the functioning of the code is given at the beginning of the code.
   After starting the code, please wait for a while  and then speak through the mike , commands like "forward"
   "left", "backwards" etc.

3. Controller file named 'Wheelchair_Astarpathfinding_LLM_controller_file.py'
   This controller uses LLM to interact with the wheel chair user. The intent in the user's conversation is deciphered
   by using OpenAI  . The keyword like "Bathroom", "Kitchen" etc. are captured and used as a command for moving to that destination.
   A star algorithm is used to find the shortest route to the destination.
   A short description on how the code works in given at the beginning of the code.
   After starting the code, please wait for a while  and then speak through the mike ,sentences like " where can i get milk in the house"
   or like " I want to relieve myself".. 
   
   Note : 1. It was found that the robotic wheelchair at times deviates from the path little bit, which I was not able to resolve.
          2. It is assumed that this is due to the error in the GPS and IMU sensor modules used in webots.
          3. While running 'Wheelchair_Astarpathfinding_LLM_controller_file.py', 
		(a) Please move the robot wheel chair to the translation position X = -4.5, Y = 3.5 , Z = 0.07, and   rotation X=0, Y=0, 		    Z=1, angle =-1.5708.
          	(b) Please speak without waiting for the prompt message in console.