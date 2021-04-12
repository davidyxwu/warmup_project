# Warmup Project
## Driving in a Square
### High level description
For this problem, I used a timing approach with the /cmd_vel topic.
I made the robot drive forward for a set number of seconds, and then had it turn counter-clockwise 90 degrees.
By going forward and turning four times total, the robot drives in a square.

### Code explanation
#### __init__
This function serves as the constructor for a node instance.
It initiates the node, sets up the Twist() publisher, and initiates the parameters (speed, etc.)
#### goForward()
This function makes the robot go forward for a set amount of seconds.
It sets the linear.x velocity and keeps publishing messages until the correct time is reached.
#### turn()
This function makes the robot turn 90 degrees to the left. At every while loop iteration,
it calculates the current angle the robot has turned with the turn velocity and the time elapsed (d=t*v).
When 90 degrees is reached, the robot stops turning.
#### driveSquare()
This function makes the robot drive in a square by driving forward and turning 4 times.
#### run()
This function continuouly runs driveSquare() until we stop the robot.

### Gif
![drive_square gif](https://github.com/davidyxwu/warmup_project/blob/main/gifs/drive_square.gif)

## Person Follower
### High level description
To follow an object, I used the robots LaserScan to determine which degree (0-360) was closest to the robot.
Then, using proportional control, I turned the robot to make sure the closest distance from the robot to object was degree 0 (front of robot).
Using proportional control, I was able to then adjust the robots speed until it was close to the object. 

### Code explanation
#### __init__
This function initiates the PersonFollower node and sets parameters in our class.

#### turn_degree()
This function takes the data from LaserScan and returns both the index of the shortest distance with respect to orientation of the robot, as well as an adjusted degree from -180 to 180 degrees the robot should turn.
If the closest distance was in the 170 to 180 degree range, the function returns 180 as the closest degree to avoid fluctations between positive and negative angular velocities.
The function also adjusts all degrees greater than 180 to be negative so the robot can turn both right and left.

#### process_scan()
This function recieves information from LaserScan.
First, it calls turn_degree() to determine which direction the robot should turn.
Then, it uses proportional control to adjust linear and angular speed so the robot follows the object.

#### run()
This function keeps the program alive.

### Gif
![wall_follower gif](https://github.com/davidyxwu/warmup_project/blob/main/gifs/person_follower.gif)

## Wall Follower
### High level description
For this problem, I used a combination of proportional control for angular velocity, as well as the /scan topic to detect walls.
I kept track of how close the wall was in two regions: the front and right side of the robot.
I wanted to keep the robot a certain distance away from the wall with regards to the right side, while turning left when
the robot approached a wall with respect to the front.

### Code explanation
#### __init__
This function serves as the constructor for a node instance.
It initiates the node, sets up the Twist() publisher and LaserScan subscriber which sends its information to the self.process_scan function.
It also initiates the parameters (distance from wall, etc.)
#### process_scan()
This function recieves callback information from the LaserScan.
It keeps track of two variables, front_min and right_min representing the minimum distances from the wall from the robot's front and right sides.
The robot is set up into 3 stages. First, the robot must find the wall. For this, we make the robot move forward and right slightly until we reach the thresholds to begin tracing the wall.
When the robot is within turning distance, we enter the turning stage and make the robot turn with a large angular velocity.
When the robot is moving forward along the wall, we keep the robot parallel with proportional control with respect to distance from the wall on the right.
#### run()
This function continuouly runs until we stop the robot.

### Gif
![wall_follower gif](https://github.com/davidyxwu/warmup_project/blob/main/gifs/wall_follower.gif)

## Challenges
Working with robots is hard. For one, all of the noise in the environment forced me to think carefully about the balance between speed and functionality.
Further, because data from LaserScan varied so much, I had to adjust my functions to be non-sensitive to small changes in readings. 
This meant splitting data from LaserScan into regions, making the proportional constant small, etc.
Another challenge I had was finding a balance between making the robot look "smooth" while taking into account edge cases. For example, in wall_follower, I had to make the robot follow the wall from any starting point. This meant that I had to make the robot adjust its motors alot, meaning the robot looked more "choppy". Finding a better balance between these two elements (perhaps with PID control) is something I want to work on in the future.
## Future Work
I would like to make my robots seem smoother and perform better with PID rather than just using proportional control, or constantly stopping and adjusting.
Another improvement I would like to make would be to make the robot more dynamic in wall_follower and person_follower.
This means adding options for the user to adjust speed etc. to make the robot more flexible.
One last improbement I would like to implement is more dynamic behavior for edge cases.
For example, I would like to make the wall_follower adjust correctly if it accidentally bumps into a wall, or in person follower, if the robot loses sight of the object.
## Takeaways
-Robot programming is hard. There are a lot of physical factors and noise to take into account, many of which I discovered while working on the project. As such, it is important to test and have strategies for handling edge cases.

-The proportional constant is very sensitive to small changes. Changing the proportional constant slightly makes a huge difference, and it is important to think about what behavior I want, and to test many values before settling on a proportional constant.

-OOP makes my code better organized, especially in robotics programming, where OOP principles arise naturally.
