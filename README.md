# smach_mission_gazebo

This project executes a simple mission using Gazebo simulator. 
In order to execute this project, follow the steps below.


- Execute the script from the project directory that launches Gazebo:

        $ ./launcher_gazebo.sh

- Wait until the following window is presented:

![Gazebo simulator interface with drone ](https://i.ibb.co/XV1hYDy/Captura-de-pantalla-de-2021-06-07-11-16-42.png)

- Open a new terminal and change directory to the project

- Execute the script that launches the Aerostack components for this project:
 
        $ ./main_launcher.sh

- Move to /configs/mission directory

        $ cd configs/mission
        
- Execute the following command to run the mission:

        $ ./simple_mission.py

- To stop the processes execute the following script:

        $ ./stop.sh

- To close the inactive terminals:

        $ killall bash

The following video illustrates how to launch the project:

[ ![Launch](https://ibb.co/w4kNCYx](https://www.youtube.com/watch?v=ypH81AnQ-7w)

The following video shows the mission execution:

[ ![Launch](https://ibb.co/9NzCJT6)](https://www.youtube.com/watch?v=sGUK1goKSyI)

