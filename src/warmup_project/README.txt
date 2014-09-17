Victoria Preston -- WarmUp Project
September 17, 2014

Behaviors Implemented: 
- People Following: Looks at things all around itself, follows the closest thing around, will maintain a certain distance away
- Wall Following: Will follow a wall at about 0.7 meters away, will detect whether the wall is on the right or left given a random orientation and proceed
- Obstacle Avoidance: Will move away from anything closer than a half meter to the robot in the opposite direction than the object
- In Controller: Wall Approach + Wall Follow

Implementation Strategy:
I drew pictures and thought about event dynamics before drafting any code.  These calculations helped me to think about the limiting cases, appropriate abstractions, and robot-world interactions.  Following this, I sketched a little pseudo-code, just in the order I believed would make sense to execute the behvior desired.  Then code was drafted.  Debugging using the command line print, propping the robot up to watch the wheels, and testing limiting cases was especially useful.

Finite State Controller:
For the finite state controller, I combined the Wall Approach and Wall Following codes.  The reason for these particular two is that I created each of the behaviors without regard for the others.  These were the only two behaviors that easily nested with eachother.  It certainly taught me to consider the other tasks of a robot before building code.  

I wrote my behaviors using functions rather than classes.  In this case, you must initialize both nodes at once in __main__, and the hand off must occur through 'returning' a value once one task was deemed 'successful'.  In my case, once the robot reached the wall, it returned its distance away, which triggered the wall following event to occur.  

Code Structure:
I used functions rather than classes because I was more familiar with these.  For each behavior function there was a data-reading function tailored for that behavior (to more effectively divide data handling and execution tasks).  

Challenges (and Triumphs):
There were some hardware challenges - unable to connect with the pi, low batteries corresponding to laggy and inaccurate motor reactions, etc.  These were frustrating because there was very little we could do about them.  

In terms of software, the challenge was simply debugging.  Either the simulator or the robot needed to be accessible many times, deploying code could take a little time, and some errors seemed to make little sense.  Text-editor errors like indent mishandling also occured.  

There were not any challenges that were particularly compelling, save for the logical challenges.  Code could be perfect, but if the logic behind the code was flawed, then there was realistically many hours spent debugging and improving on ideas.  As an individual project, it was especially difficult because while we felt comfortable pinging ideas off of eachother, we all do want to execute our own code and 'be right.'  I think there was a lot to learn in that space.

Improvements for the Future:
I would certainly put things in classes.  It would have made the finite controller much easier logistically, and may have impacted how I saw the relationships between the behaviors.  Wall Following, Obstacle Avoid, and People Follow all looked very similar by the end of the process, and had classes been involved there could have been parents 'Follower' and 'Avoider' which could have provided shared attributes between the behaviors.  

Additionally, I would like to learn the 'best methods and standards' for data handling.  I used lists and dictionaries primarily, but perhaps there is a better way that as a beginner I wouldn't necessarily think of.  

Lessons Learned:
This was a great way to learn ros basics.  I've done all of this coding in LabView before, but had never even contemplated using python for the same things.  Transferring my knowledge of one 'language' to another definitely taught me new things about Python, and some of the limits of my own thinking (in that, LabView has one way to execute something, whereas Python may facilitate a completely different way of approaching the problem).
