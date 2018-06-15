Header header

# This message defines lowest level commands to be sent to the actuator(s). 

float64[] angles             # Angle of the actuator in [rad]. 
                             # E.g. servo angle of a control surface(not angle of the surface!), orientation-angle of a thruster.      
float64[] angular_velocities # Angular velocities of the actuator in [rad/s].
                             # E.g. "rpm" of rotors, propellers, thrusters 
float64[] normalized         # Everything that does not fit the above, normalized between [-1 ... 1].