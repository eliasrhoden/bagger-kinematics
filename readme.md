# Bagger kinematics

This contains two simulations, one for only the bucket and rototilt and the second for a complete excavator (including rototilt).

Both are intended to run with an Xbox-controller plugged in and uses the MUJOCO simulator.

Both contains a special *robot_mode*, where one can control the end effector with fewer controls that a standard excavator.

# Screw theory

The model kinematics are handles with *screw theory* [https://en.wikipedia.org/wiki/Screw_theory](https://en.wikipedia.org/wiki/Screw_theory), more info about that can be found the the *modern robotics* book online [https://hades.mech.northwestern.edu/images/7/7f/MR.pdf](https://hades.mech.northwestern.edu/images/7/7f/MR.pdf).

Using the screws, it's possible to compute the *jacobian* than relates the bucket-velocities to the joint-velocities.

## Rototilt

The urdf-file was put together manually and also the kinematic-screw model in `rotoscrews.py`

The user inputs a desired pitch/tilt in *bucket-frame* and also a *global rotation*. Thus can you rotate the bucket without spilling.

https://github.com/user-attachments/assets/27518317-51de-498c-8b93-5ab314ff5605

## Bagger 

For the bagger, the URDF file was generated from fusion and also used for the screw models. (See `bagger_kinematics.py`)

The bagger simply has linear velocity inputs, i.e. along global x,y,z axises.

https://github.com/user-attachments/assets/8af81148-21c3-4dde-be9b-3ffc6247b197
