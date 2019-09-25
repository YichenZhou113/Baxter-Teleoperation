This project contains several versions of using an Xbox Controller controlling the Baxter Robot, by Laban Movement analysis. 
There are four parameters for describing a movement in Laban Analysis: body_part, kinesphere_size, direction and plane_level. I mapped the parameters into the Xbox controller and developed two modes. The continuous mode in which the Baxter would move only when the user is holding the joystick and the direct mode in which the Baxter would move once given an input from the joystick.
The '_fixed' versions chopped the trajectory into 200 pieces and use zeromq to transmit the real-time locations of different joints.
The '_norobot' versions are used for integrating with the virtual Baxter robot models in MoveIt framework or Unity.
