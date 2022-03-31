The nlp_node.py script is the script responsible for all natural language processing for the bedbot robot system.

Name: Sebastian Aegidius & Xinyu Pang & Sohyeon Im

CONFIGURATION INSTRUCTIONS
- The script is designed to run on the Baxter
- Ubuntu 18.04 required
- ROSPY and other ROS installations are required
- Websocket connection over the local network is also required in order to communicate with the base

Operating INSTRUCTIONS
- Connect to Baxter
- Obtain local IP address of parter computer for base communication
- Agree on port for IP websocket communication and update port address in the nlp_node.py script as well as
	Ridgeback partner Computer script.
- Node publishes to the topic "/nlp_node" with one of 7 states. Once script is initialized and prints "listening..." comman dcan be given.
- The script subscribes to the "/sonar_state" topic form the baxter sonar node and the main baxter arm movement topic "baxter_state_topic".
	
Modifications
- different commands can be added in the nlp_thread_func function and the variable can be changed to include more output states.
- The two call back functions are for processing the data from the sonar and baxter arm topics and must NOT be modified to ensure system works.
	
