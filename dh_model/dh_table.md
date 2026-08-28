# AQUA-ARM DH Parameter Table

The kinematic model uses the active serial chain J1-J4.

Joint J5 is mechanically coupled to J4 through the URDF mimic relationship:

q5 = -q4

## Standard DH Parameters

| Joint | a (m) | alpha (rad) | d (m) | theta |
|------|------:|------------:|------:|------:|
| J1 | 0.00 | pi/2 | 0.307 | q1 |
| J2 | 0.80 | 0 | -0.020 | q2 |
| J3 | 0.00 | pi/2 | 0.350 | q3 |
| J4 | 0.13 | 0 | 0.920 | q4 |

## Joint relationship

q5 = -q4

Therefore the gripper motion is represented by the J4 command while J5 follows through the mimic relationship.