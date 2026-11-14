'''
    pyfrc basic simulation tests are skipped because this robot uses Phoenix6
    hardware (TalonFX/CANivore) whose CAN configuration calls block in plain
    pyfrc simulation, causing robotInit to exceed the 2-second timeout.
    Deploy to real hardware or use the Phoenix Tuner X simulator instead.
'''

from pyfrc.tests import *

