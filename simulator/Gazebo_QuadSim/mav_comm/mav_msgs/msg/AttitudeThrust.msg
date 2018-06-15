Header header

geometry_msgs/Quaternion attitude    # Attitude expressed in the header/frame_id frame.
geometry_msgs/Vector3 thrust         # Thrust [N] expressed in the body frame.
                                     # For a fixed-wing, usually the x-component is used.
                                     # For a multi-rotor, usually the z-component is used.
                                     # Set all un-used components to 0.
