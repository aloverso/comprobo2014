# Warmup Project Write-Up
Anne LoVerso
9/19/2104

## Behaviors
I implemented two of given behaviors, person following and obstacle avoidance.  

### Person Following
I took note of several strategies suggested for person following.  I began with attempting to merely follow the closest detected object, but that created difficulties whenever it was near a wall or any other object that "distracted" its attention.

I followed up with several attempts to remedy this.  I tried following the object closest to the front of the robot, but distractions still occurred.  I had taken to carrying around a white sheet of paper to get the robot to follow me, because its Laser Scanner found the white easier to see than dark jeans and therefore rarely missed detecting me.  Therefore, I tried having the robot follow the brightest object it saw, of the greatest intensity.  However, it still got distracted by objects that were close by that caught its attention.

Finally, I decided to see if I could somehow have it take all these into account.  I created a function with the distance, heading, and intensity of a given object, and the robot would track the object it saw with the highest functional return value.

After several iterations of estimation, checking parameters, and re-guessing, I came up with this function:
'''
def optimize(d,a,i):
    return (i/15.0) + 3*cos(.5*a) + 1.5/(d)
'''
Where d is a distance in meters between about .2 and 3, a is an angle in degrees between 180 and -180, and i is an intensity, ranging anywhere from 10 (dark objects) to over 400 (very bright objects).  This function tries to maximize all three attributes, such that when the robot is happy with its position (a perfect .75m away from the target, facing it directly), all the parameters are in the 2 - 3 range, all close to equal.  The point of this function was to take all these parameters into account, so that in different situations, like the robot being very close to a person, but facing the wrong way, or a person being far away, but directly in front of the robot, different parameters "win" and take control of the function, so the robot will always know what to follow.  This function seems to work far better than any attribute alone.

The person follower all implements a finite state controller.  It translates between the states of "lonely" and seeking out its person.  When the robot is lonely, it means it cannot sense any objects within its "doughnut" of scan ranges.  When it is lonely, it moves forward at a reasonable speed, seeking out an object to follow.  Once an object appears on the robot's sensors, it will switch into "follow" mode and follow that object, or the object deemed most reasonable by the optimization function, until no objects are on its sensors anymore, and it switches back to lonely mode.

The code for the person follower is structured as object-oriented, where the code creates an instance of the PetRobot class, which has defined methods that control its interpretation of the laser scans and subsequent movement.

### Obstacle Avoidance

I attempted to implement obstacle avoidance through the method suggested.  I considered each obstacle within the robot's scans to be exerting a repulsive force.  The magnitude of the force would get larger the closer the robot got, and the sum of those forces was the direction of travel for the robot, with a preference for forwards motion.  It is not perfect - if there's an object directly to the front and rear, for example, the sum of forces would cancel each other out, but the preference for forwards would cause a collision.  The hope is that the forward motion would change the forces enough to cause the robot to divert its path, or that such situations wouldn't come up in an imperfect world.  

The code does not have an object-oriented structure, because it was written before that was introduced in class.  It does tend to work, albeit best when moving very slowly.  For the proportional control, I multiplied the error by a factor of 0.01, which causes the robot to move very slowly, but it hardly ever hits obstacles.  A factor of 0.1 made the robot's decisions quick enough that sometimes it did not turn enough in time to avoid obstacles.

## Challenges

I did encounter some challenges while working on this.  Most notably, the robots can be finicky, and even if something seems right in the code logic, the real world is hardly perfect.  The laser scanners were sometimes touchy in deciding whether they saw objects.  In addition, I did run into the challenge that my robot would suddenly become incredibly chaotic in its motions, following none of the programmed logic.  I spent a long time attempting to debug until I noticed on the rviz visualizer of the laser scans that the scanner was picking up readings on top of the robot itself.  The wires from connecting the Raspberry Pi to its camera were sticking up enough to be read by the laser scanner, and the robot was essentially attempting to avoid itself.  I fixed that problem by taping over the wires with paper.

I did find that my programs worked significantly better in the Gazebo simulator than in real life.  The person follower especially was almost flawless in a completely empty world other than the object it was following.  I also used the simulator to creat an elaborate obstacle course for the robot.  It did well again at low speed, but sometimes got overexcited and ran headlong at objects without turning enough to avoid them.  The problem was, I found, that the code made the force an inverse of distance, so as it got closer, the force got greater, and the robot moved faster, but the angles weren't enough to divert its path.  I tried to fix this to some extent by multiplying the angle error by two, so that the angle is more of a factor, and it improves the functionality somewhat.  Overall, though, the clear laser scans of the simulator work far better than the real world, as could be expected.

## Reflections

If I had more time, I would spend some time improving my algorithms.  I don't feel like obstacle avoidance is as robust as it could be, and that could be improved by some more work and time.  In addition, learning more about how to write a optimization function for person following could improve that behavior significantly.  The current function is a product of guess-and-check, and it could probably be improved.

In addition, I feel like the person follower could be improved by some time spent on looking for specific objects.  At the moment, it investigates each data point from the 360-degree circle separately.  Looking at clumps of distance readings as a single object could definitely improve functionality and make it more robust.

The project did spark some interesting ideas for the future.  While working on person following, it was most challenging to figure out how the robot should decide which object in its range it should follow.  It could be fun to incorporate using the camera, and having the robot follow the object wearing an orange vest, or something similar.  Adding color detection could be a fun and interesting challenge.