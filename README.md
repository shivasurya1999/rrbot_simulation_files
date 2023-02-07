# rrbot_simulation_files
 RBE500 final project
Group Assignment 
Shiva Surya Lolla,
Chaitanya Sriram Gaddipati, 
Ethan Wilke 

The “Position control” file consists of a subscriber file that reads and organizes the recorded current positions of the joints and then sends them out in a message format through a publisher

“Main Controller” takes in the message from the “position_control” node and takes desired joint values through a service. It calculates the joint efforts through PD controllers for each joint. This calculated joint effort for each joint is then sent out through a publisher, eventually being read from the Gazebo program for the simulation




