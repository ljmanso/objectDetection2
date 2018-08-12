# objectDetection2
This is a new RoboComp component that integrated in a complete robot system allows it to know where itself is and the things in its environment are, *objectDetection2* is a kind of awareness.

## Google Summer of Code 2018
This repository is the result of the 2018 Google Summer of Code project "Visual Detection Mechanisms in Mobile Robots" by Cristina Mendoza Gutiérrez. The following expanding work on RoboComp was done during GSoC 2018:
- Study the existing components to konw what we have and what we need.
- After the study, create a new component to improve the ObjectDetection component previously created.
- Start coding with simple goals, without moving the robot's neck.
- Do some test of the previous code.
- Code better solutions adding the robot's neck engines to set the system with movement.
- Test the last improves with only a table.
- Add new code to allow the robot to see more than one table.
- Test all and record it into video tests.

For more details follow this [link](https://github.com/CristinaMG/web/tree/patch-4/gsoc/2018/cristina_mg) to see all the GSOC posts and documentation.

## Work to be done
This work presents a possible form of attention for advanced robots. Although frequently not considered in robotic systems, self-awareness and attention can help developing or complementing many other tasks a robot must perform.

The system must be improved to be faster and more receptive, but it is a very interesting first approach to a much more complex model of attention. Nevertheless, the computational load of the program is very high. Parallelization of certain parts would lighten the process considerably. Along with these general improvements, other refinements can be considered:
- Add the control of other robot engines to the system, such as the base to facilitate obtaining more information by looking at other parts of the environment.
- Extend the types of objects detected by the system. This would be possible within the limits of Yolo, since it is the employed object detector.
- Add a *velocity* property to the visible objects in order to predict where the object would be in the next iteration of the process.
- Add the tracking eye movement so that the camera is able to follow an object in continuous movement in order to always keep it in its field of view without sudden jumps.
- Control the temperature of the map with differential equations, in order to avoid applying any threshold.
- Do not add or delete objects until there is certainty of presence or absence. This means that the object must appear or disappear in several consecutive frames to be added or deleted.

The ideal model of this system would observe the real environment and dynamically create its own internal model. Although many improvements are needed to get there, the system developed is an initial step in this direction.

## Results
To see the video results you can follow these links:
- [Tracking objects.](https://youtu.be/TZZbTtW21vE)
- [Alternating attention between two objects.](https://youtu.be/bkswvoK188M)
- [Mix of the previous two tests: tracking + alternation.](https://youtu.be/DoxEQk\_JEi0)
- [Changing the focus of attention from one table to another.](https://youtu.be/kT-OAfUe6FA)

Cristina Mendoza Gutiérrez.
Google Summer of Code 2018.
