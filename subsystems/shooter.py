import commands2
from commands2 import Subsystem
from phoenix6 import hardware, controls, configs, signals
import wpilib

class ShooterSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        # 1. Define your motors (Change IDs if needed)
        self.flywheel = hardware.TalonFX(30)  # Your Kraken X60
        self.feeder = hardware.TalonFX(29)    # Your Intake/Feeder Motor
        self.screw = hardware.TalonFX(28)    # Your Screw Mechanism Motor (if you have one)

        self.flywheel_requirement = commands2.Subsystem()
        self.feeder_requirement = commands2.Subsystem()
        self.screw_requirement = commands2.Subsystem()

        # 2. Configure the Kraken (Current Limits are CRITICAL)
        kraken_config = configs.TalonFXConfiguration()
        
        # Supply Limit: Protects the breaker/battery (40 Amps)
        kraken_config.current_limits.supply_current_limit = 40.0
        kraken_config.current_limits.supply_current_limit_enable = True
        
        # Stator Limit: Protects the motor from heat (110 Amps)
        kraken_config.current_limits.stator_current_limit = 90
        kraken_config.current_limits.stator_current_limit_enable = True

        slot0 = kraken_config.slot0
        slot0.k_s = 0.25  # Voltage to overcome static friction
        slot0.k_v = 0.12  # Voltage per RPS (Top speed ~12V / 100 RPS)
        slot0.k_p = 0.11  # Error correction

        # Apply configs (skip in simulation — CAN not available in pyfrc tests)
        if not wpilib.RobotBase.isSimulation():
            self.flywheel.configurator.apply(kraken_config)
            self.screw.configurator.apply(kraken_config)

    def run_flywheel(self, velocity_rps: float):
        """
        Spins the main shooter wheel.
        Using VelocityVoltage for consistent speed.
        """
        # Note: You need to tune PID for this to work perfectly!
        # For now, we use VoltageOut for testing if you haven't tuned yet.
        # self.flywheel.set_control(controls.VelocityVoltage(velocity_rps))
        
        # Simpler version for today: Voltage (0-12V)
        self.flywheel.set_control(controls.VelocityVoltage(velocity_rps)) 

    def run_feeder(self, percent: float):
        """
        Runs the intake/feeder motor.
        Range: -1.0 to 1.0
        """
        percent_screw = percent * -1.0
        self.feeder.set_control(controls.DutyCycleOut(percent))
        self.screw.set_control(controls.DutyCycleOut(percent_screw))

    def run_screw(self, percent: float):
        """
        Runs the screw mechanism motor.
        Range: -1.0 to 1.0
        """
        self.screw.set_control(controls.DutyCycleOut(percent))

    def stop_flywheel(self):
        """Turn everything off safely."""
        self.flywheel.set_control(controls.NeutralOut())
    def stop_feeder(self):
        self.feeder.set_control(controls.NeutralOut())
        self.screw.set_control(controls.NeutralOut())
        # In subsystems/shooter.py
    def stop_screw(self):
        self.screw.set_control(controls.NeutralOut())



