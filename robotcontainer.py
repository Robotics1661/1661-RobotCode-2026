#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from commands2 import cmd
from commands2.button import CommandXboxController, Trigger, CommandJoystick, JoystickButton
from commands2.sysid import SysIdRoutine

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6 import controls, hardware
from phoenix6 import swerve
from phoenix6 import configs
import phoenix6
import ntcore
from wpimath.controller import PIDController
import math

from subsystems.shooter import ShooterSubsystem

from pathplannerlib.auto import NamedCommands, PathPlannerAuto

from wpilib import Joystick, SendableChooser, RobotBase, Timer
from wpilib.shuffleboard import Shuffleboard, BuiltInWidgets
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._max_speed = (
            1.0 * TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.5
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.4
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )


        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(self._max_speed)


        # Intake Pivot and Intake
        self.motor_intake_pivot = hardware.TalonFX(38)
        self.motor_intake = TalonFX(39)

        # Shooter and Shooter Intake
        # PID for distance (Forward/Back)
        self.dist_controller = PIDController(1.5, 0, 0) 
                # PID for alignment (Left/Right rotation)
        self.rot_controller = PIDController(0.05, 0, 0)
        self.rot_controller.setSetpoint(0) # Target crosshair at 0

        self._robot_drive = swerve.requests.RobotCentric() \
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        
        self.dist_controller.setTolerance(0.05) # Stop within 5cm (approx 2 inches)
        self.rot_controller.setTolerance(1.0)   # Stop within 1 degree


        self.limelight_table = ntcore.NetworkTableInstance.getDefault().getTable("limelight")


        # Pre-define Control Requests
        self.neutral_stop = phoenix6.controls.static_brake.StaticBrake()

        # 1. Setup PID gains (crucial for holding position)
        pivot_config = configs.TalonFXConfiguration()
        slot0 = pivot_config.slot0
        slot0.k_p = 12.0 # Adjust based on your mechanism's weight
        slot0.k_v = 0.12 # Example value

        if not RobotBase.isSimulation():
            self.motor_intake_pivot.configurator.apply(pivot_config)

        # 2. Use PositionVoltage instead of StaticBrake
        self.position_hold = controls.PositionVoltage(0) 


        self.pivot_forward = phoenix6.controls.DutyCycleOut(0.2)
        self.pivot_reverse = phoenix6.controls.DutyCycleOut(-0.2)
        self.intake_speed = phoenix6.controls.DutyCycleOut(-0.4)
        self.intake_speed_out = phoenix6.controls.DutyCycleOut(0.4)

        self.intake_shooter_speed = phoenix6.controls.DutyCycleOut(.2)
        self.shooter_speed = phoenix6.controls.DutyCycleOut(-.4)
        self.shooter_speed_target = 36.75  # Starting speed in RPS
        self.shooter_speed_target_out = -20.0
        self.shooter = ShooterSubsystem()

        # Register named commands for PathPlanner (must be before drivetrain/AutoBuilder)
        NamedCommands.registerCommand("RunFlywheel", commands2.InstantCommand(
            lambda: self.shooter.run_flywheel(self.shooter_speed_target), self.shooter
        ))
        NamedCommands.registerCommand("StopFlywheel", commands2.InstantCommand(
            self.shooter.stop_flywheel, self.shooter
        ))
        NamedCommands.registerCommand("RunFeeder", commands2.InstantCommand(
            lambda: self.shooter.run_feeder(0.45), self.shooter
        ))
        NamedCommands.registerCommand("StopFeeder", commands2.InstantCommand(
            self.shooter.stop_feeder, self.shooter
        ))
        NamedCommands.registerCommand("RunIntake", commands2.InstantCommand(
            lambda: self.motor_intake.set_control(self.intake_speed)
        ))
        NamedCommands.registerCommand("StopIntake", commands2.InstantCommand(
            lambda: self.motor_intake.set_control(self.neutral_stop)
        ))        
        NamedCommands.registerCommand("RunScrew", commands2.InstantCommand(
            lambda: self.shooter.run_screw(-0.35), self.shooter))
        NamedCommands.registerCommand("StopScrew", commands2.InstantCommand(
            lambda: self.shooter.stop_screw(), self.shooter
        ))
        _stall_start = [None]  # mutable container for closure

        def _lower_init():
            _stall_start[0] = None

        def _lower_execute():
            self.motor_intake_pivot.set_control(self.pivot_reverse)

        def _lower_finished():
            velocity = abs(self.motor_intake_pivot.get_velocity().value)
            now = Timer.getFPGATimestamp()
            if velocity < 1.0:  # < 1 RPS treated as stalled
                if _stall_start[0] is None:
                    _stall_start[0] = now
                elif now - _stall_start[0] >= 0.15:
                    return True
            else:
                _stall_start[0] = None
            return False

        def _lower_end(_interrupted):
            pass  # hold position is applied by finallyDo after the full chain

        NamedCommands.registerCommand("LowerIntake",
            commands2.FunctionalCommand(
                _lower_init, _lower_execute, _lower_end, _lower_finished
            ).andThen(
                # Stall detected — lift back up briefly to relieve pressure
                commands2.RunCommand(
                    lambda: self.motor_intake_pivot.set_control(self.pivot_forward)
                ).withTimeout(0.1)
            ).finallyDo(
                lambda _: self.motor_intake_pivot.set_control(
                    self.position_hold.with_position(self.motor_intake_pivot.get_position().value)
                )
            )
        )
 

        # Configure BRAKE mode
        pivot_cfg = phoenix6.configs.TalonFXConfiguration()
        pivot_cfg.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.BRAKE


        self.xbox_controller = CommandXboxController(0)
        self.joystick = CommandJoystick(1) 
        

        self.drivetrain = TunerConstants.create_drivetrain()
        NamedCommands.registerCommand("Jiggle",
            self.drivetrain.apply_request(
                lambda: self._robot_drive
                .with_velocity_x(0)
                .with_velocity_y(0.6 * math.sin(Timer.getFPGATimestamp() * 2 * math.pi * 8))
                .with_rotational_rate(0))
        )

        # Auto chooser — Left or Right
        self._auto_chooser = SendableChooser()
        self._auto_chooser.setDefaultOption("Left", "Blue_left")
        self._auto_chooser.addOption("Right", "Blue_right")

        auto_tab = Shuffleboard.getTab("Auto")
        auto_tab.add("Auto Chooser", self._auto_chooser).withSize(2, 1).withPosition(0, 0)

        teleop_tab = Shuffleboard.getTab("Teleoperated")
        self._flywheel_entry = (
            teleop_tab.add("Flywheel Speed (RPS)", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(2, 1)
            .withPosition(0, 0)
            .getEntry()
        )



        self.configureButtonBindings()

    def get_limelight_align_command(self):
        """
        Command to align the robot to an AprilTag at a distance of 60 inches (1.524m).
        Triggered by Joystick Button 1.
        """
        # --- CONFIGURATION (SI UNITS: METERS & RADIANS) ---
        TARGET_HEIGHT_M = 1.124      # 44.25 inches (AprilTag height)
        CAMERA_HEIGHT_M = 0.22125    # Camera lens height from floor
        MOUNT_ANGLE_DEG = 30.0       # Degrees camera is tilted UP from horizontal
        GOAL_DISTANCE_M = 2      # 88.74 inches (Desired stopping point)
        
        # Speed Limits for Safety
        MAX_DRIVE_MPS = 1.5          # Max 1.5 meters per second
        MAX_ROT_RADPS = 2.0          # Max ~1/3 rotation per second
        OFF_CENTER_OFFSET_DEG = 0

        def drive_logic():
            # Retrieve values from Limelight NetworkTable
            tv = self.limelight_table.getNumber("tv", 0) # Target Valid
            tx = self.limelight_table.getNumber("tx", 0) # Horizontal Offset
            ty = self.limelight_table.getNumber("ty", 0) # Vertical Offset

            # 1. Check if we actually see a target
            if tv > 0:
                # 2. Distance Calculation (Trigonometry)
                angle_to_goal_rad = math.radians(MOUNT_ANGLE_DEG + ty)
                
                # Prevent division by zero
                denom = math.tan(angle_to_goal_rad)
                if abs(denom) < 0.001: denom = 0.001
                
                current_distance_m = (TARGET_HEIGHT_M - CAMERA_HEIGHT_M) / denom

                # 3. PID Calculations
                # Forward/Back speed (X) based on distance error
                drive_speed = self.dist_controller.calculate(current_distance_m, GOAL_DISTANCE_M)
                
                # Rotational speed based on horizontal error (Aiming for 0)
                rot_speed = self.rot_controller.calculate(tx, OFF_CENTER_OFFSET_DEG)

                # 4. Safety Clamping (Prevents robot from zooming too fast)
                if abs(current_distance_m - GOAL_DISTANCE_M) < 0.1:
                    drive_speed = 0

                # If we are within 1 degree of our rotation goal, stop spinning
                if abs(tx - OFF_CENTER_OFFSET_DEG) < .25:
                    rot_speed = 0
                drive_speed = max(min(drive_speed, MAX_DRIVE_MPS), -MAX_DRIVE_MPS)
                rot_speed = max(min(rot_speed, MAX_ROT_RADPS), -MAX_ROT_RADPS)

                # Optional: Log distance to Dashboard for easy debugging
                # wpilib.SmartDashboard.putNumber("Limelight/Distance Meters", current_distance_m)

                # 5. Output Robot-Relative Request
                # (X is forward/back, Rot is spinning)
                return self._robot_drive.with_velocity_x(-drive_speed).with_rotational_rate(rot_speed)
            
            else:
                # If target is lost, stop movement immediately
                return self._brake

        # Apply the logic to the drivetrain
        return self.drivetrain.apply_request(drive_logic)


    def configureButtonBindings(self) -> None:

        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: self._drive
                .with_velocity_x(-self.joystick.getY() * self._max_speed)      # Forward/Back
                .with_velocity_y(-self.joystick.getX() * self._max_speed)      # Left/Right
                .with_rotational_rate(-self.joystick.getZ() * self._max_angular_rate) # Twist
            )
        )


        # Wiggle (Button 12) — fast side-to-side oscillation to settle balls, no rotation
        self.joystick.button(12).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._robot_drive
                .with_velocity_x(0)
                .with_velocity_y(0.5 * math.sin(Timer.getFPGATimestamp() * 2 * math.pi * 8))
                .with_rotational_rate(0)
            )
        )

        # Brake (Button 2 / Trigger)
        self.joystick.button(2).whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        # Assuming Button 1 is the trigger on your joystick

        self.joystick.button(1).whileTrue(self.get_limelight_align_command())

        # Intake
        self.joystick.button(4).whileTrue(commands2.RunCommand(
            lambda: self.motor_intake.set_control(self.intake_speed)
            ).finallyDo(lambda interrupted: self.motor_intake.set_control(self.neutral_stop))
            )
        self.joystick.button(5).whileTrue(commands2.RunCommand(    
            lambda: self.motor_intake.set_control(self.intake_speed_out)
            ).finallyDo(lambda interrupted: self.motor_intake.set_control(self.neutral_stop))
            )
        
        #Zeroing the pivot with a button (Button 7) self.drivetrain.get_pigeon2().set_yaw(0)

        
        self.joystick.button(7).onTrue(
            commands2.InstantCommand(lambda: self.drivetrain.seed_field_centric(), self.drivetrain)
        )
        
        # Pivot Intake
        self.xbox_controller.povUp().whileTrue(commands2.RunCommand(
            lambda: self.motor_intake_pivot.set_control(self.pivot_forward)
            ).finallyDo(
            lambda interrupted: self.motor_intake_pivot.set_control(
                # Get current position and hold it
                self.position_hold.with_position(self.motor_intake_pivot.get_position().value)
            ))
        )

        # D-Pad DOWN: Move Pivot Backward
        self.xbox_controller.povDown().whileTrue(commands2.RunCommand(
            lambda: self.motor_intake_pivot.set_control(self.pivot_reverse)
            ).finallyDo(
            lambda interrupted: self.motor_intake_pivot.set_control(
                # Get current position and hold it
                self.position_hold.with_position(self.motor_intake_pivot.get_position().value)
            ))
        )


        
        self.xbox_controller.povRight().onTrue(
            commands2.cmd.runOnce(lambda: setattr(self, 'shooter_speed_target', 39.5))
            )

        
        self.xbox_controller.rightBumper().onTrue(
            commands2.cmd.runOnce(lambda: setattr(self, 'shooter_speed_target', self.shooter_speed_target + .25))
            )

        self.xbox_controller.leftBumper().onTrue(
            commands2.cmd.runOnce(lambda: setattr(self, 'shooter_speed_target', self.shooter_speed_target - .25))
            )

        self.xbox_controller.leftTrigger().whileTrue(
            commands2.RunCommand(
                lambda: self.shooter.run_flywheel(self.shooter_speed_target), 
                self.shooter
            )
            ).onFalse(commands2.InstantCommand(self.shooter.stop_flywheel))
        self.xbox_controller.x().whileTrue(
            commands2.RunCommand(
            lambda: self.shooter.run_flywheel(self.shooter_speed_target_out),
            self.shooter
            )).onFalse(commands2.InstantCommand(self.shooter.stop_flywheel))
                # Button for Feeder (Right Trigger)

        self.xbox_controller.rightTrigger().whileTrue(
            commands2.RunCommand(
                lambda: self.shooter.run_feeder(0.45), 
                self.shooter # Only locks the feeder
            )
            ).onFalse(commands2.InstantCommand(self.shooter.stop_feeder))
        
        self.xbox_controller.a().whileTrue(
            commands2.RunCommand(
                lambda: self.shooter.run_screw(-0.3), 
                self.shooter # Only locks the feeder
            )
            ).onFalse(commands2.InstantCommand(self.shooter.stop_feeder))        
        
        self.xbox_controller.x().whileTrue(
            commands2.RunCommand(
                lambda: self.shooter.run_feeder(-0.3), 
                self.shooter # Only locks the feeder
            )
            ).onFalse(commands2.InstantCommand(self.shooter.stop_feeder))

        

    def stopAll(self):
        """Stop all mechanism motors. Called on transition out of auto."""
        self.shooter.stop_flywheel()
        self.shooter.stop_feeder()
        self.motor_intake.set_control(self.neutral_stop)

    def getAutonomousCommand(self):
        name = self._auto_chooser.getSelected()
        if name:
            return PathPlannerAuto(name)
        return None

    def get_flywheel_velocity(self):
        # Returns velocity in Rotations per Second (RPS)
        return self.shooter_speed_target

