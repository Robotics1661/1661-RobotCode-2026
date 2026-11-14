#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import commands2
import typing
import math
from wpilib import SmartDashboard, DriverStation

from robotcontainer import RobotContainer

from phoenix6 import HootAutoReplay



class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """


    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()


        # log and replay timestamp and joystick data
        self._time_and_joystick_replay = (
            HootAutoReplay()
            .with_timestamp_replay()
            .with_joystick_replay()
        )

    def robotPeriodic(self) -> None:
        """This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating."""
        commands2.CommandScheduler.getInstance().run()
        flywheel_speed = self.container.get_flywheel_velocity()
        self.container._flywheel_entry.setDouble(flywheel_speed)

        alliance = DriverStation.getAlliance()
        if alliance == DriverStation.Alliance.kRed:
            alliance_str = "Red"
        elif alliance == DriverStation.Alliance.kBlue:
            alliance_str = "Blue"
        else:
            alliance_str = "Unknown"
        location = DriverStation.getLocation()
        SmartDashboard.putString("Alliance", alliance_str)
        SmartDashboard.putNumber("Station", location if location is not None else -1)


        self._time_and_joystick_replay.update()
        # Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        # commands, running already-scheduled commands, removing finished or interrupted commands,
        # and running subsystem periodic() methods.  This must be called from the robot's periodic
        # block in order for anything in the Command-based framework to work.

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        pass

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.autonomousCommand = self.container.getAutonomousCommand()
        if self.autonomousCommand:
            self.autonomousCommand.schedule()
        pass

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        pass

    def teleopInit(self) -> None:
        # Cancel autonomous and stop all mechanism motors
        if hasattr(self, "autonomousCommand") and self.autonomousCommand:
            self.autonomousCommand.cancel()
        self.container.stopAll()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        pass

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()