// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.IO.TurretIO;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import static edu.wpi.first.units.Units.*;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;
import edu.wpi.first.wpilibj2.command.*;

public class s_Hood extends SubsystemBase implements CheckableSubsystem {
  /** Creates a new s_Hood. */

  private boolean initialized = false;
  private static s_Hood m_Instance;

  public s_Hood() {
    initialized = true;
  }

  private final TurretIO turretSimulation = TurretIO.getInstance();
  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable driveStateTable = networkTable.getTable("DriveState/TurretTurntable");
  private final DoublePublisher hoodDeg = driveStateTable.getDoubleTopic("HoodDegrees").publish();

  TalonFX hoodMotor = new TalonFX(40);

  SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(12, 0, 0, DegreesPerSecond.of(135), DegreesPerSecondPerSecond.of(90))
      // Configure Motor and Mechanism propertes
      .withGearing(new MechanismGearing(30 / 16, 40 / 20, 34 / 16, 210 / 40))
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(false)
      // Setup Telemetry\
      .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
      // Power Optimization
      .withStatorCurrentLimit(Amps.of(15))
      .withClosedLoopRampRate(Seconds.of(0.0))

      .withOpenLoopRampRate(Seconds.of(0.0));

  SmartMotorController motor = new TalonFXWrapper(hoodMotor,
      DCMotor.getMinion(1),
      motorConfig);

  PivotConfig m_config = new PivotConfig(motor)
      .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
      .withHardLimit(Degrees.of(0), Degrees.of(50)) // Hard limit bc wiring prevents infinite spinning
      .withSoftLimits(Degrees.of(0), Degrees.of(50))
      .withTelemetry("Hood", TelemetryVerbosity.HIGH) // Telemetry
      .withMOI(KilogramSquareMeters.of(0.04475326));

  private Pivot hood = new Pivot(m_config);

  public boolean checkSubsystem() {
    return getInitialized();
  }

  public boolean getInitialized() {
    return initialized;
  }

  public void setDegrees(double actualDegrees) {
    actualDegrees %= 360;
    
    CommandScheduler.getInstance().schedule(
        hood.setAngle(Degrees.of(actualDegrees)));
  }

  public void stop() {
    CommandScheduler.getInstance().schedule(
        hood.set(0));
  }

  public static s_Hood getInstance() {
    if (m_Instance == null) {
      m_Instance = new s_Hood();
    }
    return m_Instance;
  }

  @Override
  public void periodic() {
    hood.updateTelemetry();
    turretSimulation.setHoodDegrees(hood.getAngle().in(Degrees));
    hoodDeg.set(hood.getAngle().in(Degrees));
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    hood.simIterate();
  }
}