# rrbot_simulation_files
 RBE500 final part2
Group Assignment – Part 2
Shiva Surya Lolla
Chaitanya Sriram Gaddipati 
Ethan Wilke 

Part 1.) The Code 

The “Position control” file consists of a subscriber file that reads and organizes the recorded current positions of the joints and then sends them out in a message format through a publisher. Below is its code. (Please check code comments below for documentation)


“Main Controller” takes in the message from the “position_control” node and takes desired joint values through a service. It calculates the joint efforts through PD controllers for each joint. This calculated joint effort for each joint is then sent out through a publisher, eventually being read from the Gazebo program for the simulation. Below is its code. (Please check code comments below for documentation)









PART 2.) The Results
Initial condition of robot 


Joint Position Reference 1
Joint 1 reference value: 0.5 
Joint 2 reference value: 1.57
Joint 3 reference value: -0.2
Screenshot attached below for service call 





Graphs generated for a period of 60 seconds with sampling time 0.1 seconds.  As we can see, the final values are close to the reference values 

Figure: Joint 1 graph (top left), Joint 2 graph (top right), Joint 3 graph (bottom)

Final condition of the robot 


Joint Position Reference 2
Joint 1 reference value: -0.8
Joint 2 reference value: 2.05
Joint 3 reference value: -0.3
Screenshot attached below for service call 




Graphs generated for a period of 60 seconds with sampling time 0.1 seconds.  As we can see, the final values are close to the reference values 


Figure: Joint 1 graph (top left), Joint 2 graph (top right), Joint 3 graph (bottom)

Final condition of the robot 






Joint Position Reference 3
Joint 1 reference value: 1.5
Joint 2 reference value: 1
Joint 3 reference value: -0.1
Screenshot attached below for service call 





Graphs generated for a period of 60 seconds with sampling time 0.1 seconds.  As we can see, the final values are close to the reference values 


Figure: Joint 1 graph (top left), Joint 2 graph (top right), Joint 3 graph (bottom)
Final Position of the robot 



