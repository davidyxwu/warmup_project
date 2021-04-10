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
TODO

### Code explanation
#### __init__
TODO

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
This function recieves from callback information from the LaserScan.
It keeps track of two variables, front_min and right_min representing the minimum distances from the wall from the robot's front and right sides.
The robot is set up into 3 stages. First, the robot must find the wall. For this, we make the robot move forward and right slightly until we reach the thresholds to begin tracing the wall.
When the robot is within turning distance, we enter the turning stage and make the robot turn with a large angular velocity.
When the robot is moving forward along the wall, we keep the robot parallel with proportional control with respect to distance from the wall on the right. 
#### run()
This function continuouly runs until we stop the robot.

### Gif
![wall_follower gif](https://github.com/davidyxwu/warmup_project/blob/main/gifs/wall_follower.gif)

## Challenges
TODO
## Future Work
TODO
## Takeaways
TODO
