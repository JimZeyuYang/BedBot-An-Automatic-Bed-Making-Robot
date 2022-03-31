# BedBot: An-Automatic-Bed-Making-Robot

BedBot provides independent assistance and human-robot interaction (HRI) in the hospitality industry using a Baxter robot mounted on a Ridgeback moving base. The main aim of this project is to create a robot to help nurses in making the bed and tidy up sheets after patient leaves. This report focuses on presenting the robot’s functionalities and explaining the development process.

## Description
The whole system runs on ROS Melodic on Ubuntu 18.04.

The code and data for each function of the robot are in each corresponding folder.

<br>
Here is the overall process:

![process](https://user-images.githubusercontent.com/66956640/161036931-52598f8c-9e37-4812-b414-19560bbc309a.png)

## Authors


Anthony Youssef and Xinyu Pang were responsible for blanket corner detection using Realsense RGBD camera

Jiaming Huang and Zeyu Yang were responsible for the hand-eye calibration

Zhiyuan You was responsible for controlling the Robotiq gripper

Yulin Shen was responsible for arm movement and motion planning

Zile Yang and Jing Wu were responsible for controlling the movement of the Ridgeback base

Xinping Chen was responsible for developing the graphical user interface

Sohyeon Im and Sebastian Aegidius and Xinuy Pang were responsible for developing the speech processing

Sebastian Aegidius and Konrad Seliger were responsible safety and collision aviodance


## License
[MIT](https://choosealicense.com/licenses/mit/)
