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
Gif of driving in a square:
![drive_square gif]
(/gifs/drive_square.gif)
