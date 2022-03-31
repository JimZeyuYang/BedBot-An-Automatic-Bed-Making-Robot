SONAR FOR BAXTER ROBOT:
The sonar_base_baxter.py script is the script that enables sonar naviagtion and safety-zoning for the baxter robot.

Name: Sebastian Aegidius

CONFIGURATION INSTRUCTIONS
- The script is designed to run on the Baxter
- Ubuntu 18.04 required
- ROSPY and other ROS installations are required
- Websocket connection over the local network is also required in order to communicate with the base

Operating INSTRUCTIONS
- Connect to Baxter
- Obtain local IP address of parter computer for base communication
- Sript automactically start spublishing to its own topic calles "/sonar_state".
- Execute script and monitor live printouts of state change; 0 for nothing too close and 1 for too close.
- The topic "/sonar_state" is visible to all on the baxter network.
	
Modifications
- Buffer size and filters can be adjusted depending on the enviornment in the top variables, makred with comments. 
	- Change Inner_thershold and Outer_threshold for this
- starting angle for where the sonars start actively reading the distance can be modified 
	- Variable to change is start_angle
	


<br>
<br>

SONAR FOR RIDGEBACK:
The sonar_base_operation.py script is the only necessary script to enable sonar naviagtion and safety-zoning for the base.

Name: Konrad Seliger

CONFIGURATION INSTRUCTIONS
- The script is designed to run on the Baxter
- Ubuntu 18.04 required
- ROSPY and other ROS installations are required
- Websocket connection over the local network is also required in order to communicate with the base

Operating INSTRUCTIONS
- Connect to Baxter
- Obtain local IP address of parter computer for base communication
- Agree on port for IP websocket communication and update port address in the sonar_base_operation script as well as
	Ridgeback partner Computer
- Execute script and monitor live printouts on commands and domainant angles in the shell to verify correct connection
- Execute partner script on the Ridgeback host computer to receive commands. If necessary perform a printout of socket received
	messages.
	
Modifications
- Buffer size and filters can be adjusted depending on the enviornment in the callback(data) function. 
- IP address and port can be modified in s.connect((___)) procedure
- calcAngle() function can me modified in the commented parts to change angle representations from -180°,  +180° to 360° field. 
	- THIS IS NOT RECOMMENDED FOR RIDGEBACK OPERATION
	
