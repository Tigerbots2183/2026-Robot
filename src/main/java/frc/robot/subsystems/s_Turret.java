// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import static edu.wpi.first.units.Units.*;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.IO.TurretIO;
import frc.robot.generated.TunerConstants;

public class s_Turret extends SubsystemBase implements CheckableSubsystem {
  /** Creates a new s_Turret. */
  private boolean initialized = false;
  private static s_Turret m_Instance;
  private double previousDegrees = 0;

  private double previousTimestamp = 0;
  private double previousYamsDegrees = 0;

  public Supplier<Boolean> inaccurate = () -> false;
  private final TurretIO turretSimulation = TurretIO.getInstance();
  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable driveStateTable = networkTable.getTable("TurretState");
  private final DoublePublisher yamsRotation = driveStateTable.getDoubleTopic("yamsRotation").publish();
  private final DoublePublisher yamsVelocity = driveStateTable.getDoubleTopic("yamsVelocity").publish();
  private final DoublePublisher degreesOff = driveStateTable.getDoubleTopic("degreesOff").publish();

  private final StructPublisher<Pose3d> yamsAngleShowerPose = driveStateTable
      .getStructTopic("yamsAngleShowerPose", Pose3d.struct).publish();

  private final StructPublisher<Pose3d> actualAngleShowerPose = driveStateTable
      .getStructTopic("actualAngleShowerPose", Pose3d.struct).publish();

  private Supplier<Pose2d> robotPose = () -> TunerConstants.getInstance().getState().Pose;

  TalonFX turretMotor = new TalonFX(3);

  double[] ratio = { 144 / 15, 5, 1.12};

  SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withSimClosedLoopController(16.0, 0.0, .6, DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(1000))
      // 20,0,0.6
      .withClosedLoopController(10.0, 0.0, 0, DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(1000))
      // Configure Motor and Mechanism properties
      .withGearing(new MechanismGearing(new GearBox(ratio)))
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(false)
      .withFeedforward(new SimpleMotorFeedforward(1.1, 0.0, 0.0))

      // 0.0,5.5`
      // Setup Telemetry
      .withTelemetry("TurretMotor", TelemetryVerbosity.LOW)
      // Power Optimization
      .withStatorCurrentLimit(Amps.of(60))
      .withClosedLoopRampRate(Seconds.of(0.0))

      .withOpenLoopRampRate(Seconds.of(0.0));
  SmartMotorController motor = new TalonFXWrapper(turretMotor,
      DCMotor.getKrakenX60(1),
      motorConfig);

  PivotConfig m_config = new PivotConfig(motor)
      .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
      .withHardLimit(Degrees.of(0), Degrees.of(360)) // Hard limit bc wiring prevents infinite spinning
      .withSoftLimits(Degrees.of(0), Degrees.of(360))
      .withTelemetry("Turret", TelemetryVerbosity.LOW) // Telemetry
      .withMOI(yams.units.YUnits.PoundSquareInches.of(362.787082)); // MOI Calculation

  private Pivot turret = new Pivot(m_config);

  private double angle = 0;
  private Command angleCommand = turret.setAngle(() -> Degrees.of(angle)).ignoringDisable(true);

  public void stop() {
    turret.setVoltage(Voltage.ofBaseUnits(0.0, Millivolt));
  }

  @Override
  public boolean checkSubsystem() {
    return getInitialized();
  }

  @Override
  public boolean getInitialized() {
    return initialized;
  }

  public void setDegreeCommand() {
    CommandScheduler.getInstance().schedule(angleCommand);
  }
  double degrees;
  public void setDegrees(DoubleSupplier degreesSupplier) {
   degrees = degreesSupplier.getAsDouble();
    if (Math.abs(degrees - turret.getAngle().in(Degrees)) <= 20) {
      inaccurate = () -> false;
    } else {
      inaccurate = () -> true;
    }

    degreesOff.set(Math.abs(degrees - turret.getAngle().in(Degrees)));

    if (degrees == previousDegrees) {
      return;
    } else {

      this.angle = degrees;
    }

    actualAngleShowerPose.set(new Pose3d(robotPose.get().getTranslation().getX(),
        robotPose.get().getTranslation().getY(), 1,
        new Rotation3d(0.0, 0.0, Rotation2d.fromDegrees(degrees).plus(robotPose.get().getRotation()).getRadians())));

    yamsVelocity
        .set((turret.getAngle().in(Degrees) - previousYamsDegrees) / (Timer.getTimestamp() - previousTimestamp));

    previousYamsDegrees = turret.getAngle().in(Degrees);
    previousTimestamp = Timer.getTimestamp();
    // turretSimulation.setTurretDegrees(degrees);
  }

  public void setDegrees(double degrees) {
    if (Math.abs(degrees - turret.getAngle().in(Degrees)) <= 20) {
      inaccurate = () -> false;
    } else {
      inaccurate = () -> true;
    }

    degreesOff.set(Math.abs(degrees - turret.getAngle().in(Degrees)));

    if (degrees == previousDegrees) {
      return;
    } else {

      this.angle = degrees;
    }

    actualAngleShowerPose.set(new Pose3d(robotPose.get().getTranslation().getX(),
        robotPose.get().getTranslation().getY(), 1,
        new Rotation3d(0.0, 0.0, Rotation2d.fromDegrees(degrees).plus(robotPose.get().getRotation()).getRadians())));

    yamsVelocity
        .set((turret.getAngle().in(Degrees) - previousYamsDegrees) / (Timer.getTimestamp() - previousTimestamp));

    previousYamsDegrees = turret.getAngle().in(Degrees);
    previousTimestamp = Timer.getTimestamp();
    // turretSimulation.setTurretDegrees(degrees);
  }

  public Command getDegreeSetter(double degrees) {
    // Returns the command that sets current degrees to turret

    return turret.setAngle(Degrees.of(degrees))
        .until(() -> turret.getAngle().in(Degrees) < degrees + 10 && turret.getAngle().in(Degrees) > degrees - 10);
  }

  public Command getDegreeSetter(DoubleSupplier degreesSupplier) {
    // Returns the command that sets current degrees to turret
    Double degrees = degreesSupplier.getAsDouble();
    return turret.setAngle(Degrees.of(degrees))
        .until(() -> turret.getAngle().in(Degrees) < degrees + 10 && turret.getAngle().in(Degrees) > degrees - 10);
  }

  public static s_Turret getInstance() {
    if (m_Instance == null) {
      m_Instance = new s_Turret();
    }
    return m_Instance;
  }

  public s_Turret() {

    initialized = true;
  }

  public void runSYSID() {
    turret.sysId(Volts.of(12), Volts.of(0.5).per(Second), Seconds.of(12));
  }

  @Override
  public void periodic() {
    turret.updateTelemetry();
    yamsRotation.set(turret.getAngle().in(Degree));
    turretSimulation.setTurretDegrees(turret.getAngle().in(Degree));
    yamsAngleShowerPose.set(new Pose3d(robotPose.get().getTranslation().getX(), robotPose.get().getTranslation().getY(),
        1,
        new Rotation3d(0.0, 0.0,
            Rotation2d.fromDegrees(turret.getAngle().in(Degree)).plus(robotPose.get().getRotation()).getRadians())));
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }
}