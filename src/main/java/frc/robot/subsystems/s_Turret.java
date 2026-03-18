// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
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
import edu.wpi.first.math.controller.ArmFeedforward;
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
  private final DoublePublisher pubOffset = driveStateTable.getDoubleTopic("Offset").publish();

  private final StructPublisher<Pose3d> yamsAngleShowerPose = driveStateTable
      .getStructTopic("yamsAngleShowerPose", Pose3d.struct).publish();

  private final StructPublisher<Pose3d> actualAngleShowerPose = driveStateTable
      .getStructTopic("actualAngleShowerPose", Pose3d.struct).publish();

  private Supplier<Pose2d> robotPose = () -> TunerConstants.getInstance().getState().Pose;

  // private Supplier<Pose3d> angleShowerPose = () -> new
  // Pose3d(robotPose.get()).transformBy(new Transform3d(0,0,1,new Rotation3d()));

  boolean isSim = RobotBase.isSimulation();

  TalonFX turretMotor = new TalonFX(3);

  double[] ratio = { 144 / 15, 5, 1.085};

  SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // 99.0, 0.0, .6
      .withClosedLoopController(30, 0.0, 3, DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(1440))
      .withVendorConfig(new TalonFXConfiguration().withSlot0(new Slot0Configs().withStaticFeedforwardSign(
          StaticFeedforwardSignValue.UseClosedLoopSign)))
      // Configure Motor and Mechanism properties
      .withGearing(new MechanismGearing(new GearBox(ratio)))
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(false)
      .withFeedforward(new SimpleMotorFeedforward(0.2,9, 0.0))    
      // 0.0,5.5`
      // Setup Telemetry
      .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
      // Power Optimization
      .withSupplyCurrentLimit(Amps.of(60));
      // .withClosedLoopRampRate(Seconds.of(0.0))
      // .withOpenLoopRampRate(Seconds.of(0.0));
  SmartMotorController motor = new TalonFXWrapper(turretMotor,
      DCMotor.getKrakenX60(1),
      motorConfig);

  PivotConfig m_config = new PivotConfig(motor)
      .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
      .withHardLimit(Degrees.of(-360), Degrees.of(360)) // Hard limit bc wiring prevents infinitpe spinning
      .withSoftLimits(Degrees.of(-360), Degrees.of(360))
      .withTelemetry("Turret", TelemetryVerbosity.HIGH) // Telemetry
      .withMOI(yams.units.YUnits.PoundSquareInches.of(362.787082)); // MOI Calculation
//
  private Pivot turret = new Pivot(m_config);
  private DoubleSupplier offset = () -> 0;
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
    motor.startClosedLoopController();
    CommandScheduler.getInstance().schedule(angleCommand);
  }

  double degrees;

  public void setDegrees(DoubleSupplier degreesSupplier) {
    degrees = degreesSupplier.getAsDouble();
    degrees += offset.getAsDouble();
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

    if (isSim) {
      actualAngleShowerPose.set(new Pose3d(robotPose.get().getTranslation().getX(),
          robotPose.get().getTranslation().getY(), 1,
          new Rotation3d(0.0, 0.0, Rotation2d.fromDegrees(degrees).plus(robotPose.get().getRotation()).getRadians())));
    }
    yamsVelocity
        .set((turret.getAngle().in(Degrees) - previousYamsDegrees) / (Timer.getTimestamp() - previousTimestamp));

    previousYamsDegrees = turret.getAngle().in(Degrees);
    previousTimestamp = Timer.getTimestamp();
    // turretSimulation.setTurretDegrees(degrees);
  }

  public void setDegrees(double degrees) {
    degrees += offset.getAsDouble();
    if (Math.abs(degrees - turret.getAngle().in(Degrees)) <= 10) {
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

    if (isSim) {
      actualAngleShowerPose.set(new Pose3d(robotPose.get().getTranslation().getX(),
          robotPose.get().getTranslation().getY(), 1,
          new Rotation3d(0.0, 0.0, Rotation2d.fromDegrees(degrees).plus(robotPose.get().getRotation()).getRadians())));

    }

    yamsVelocity
        .set((turret.getAngle().in(Degrees) - previousYamsDegrees) / (Timer.getTimestamp() - previousTimestamp));

    previousYamsDegrees = turret.getAngle().in(Degrees);
    previousTimestamp = Timer.getTimestamp();
    // turretSimulation.setTurretDegrees(degrees);
  }

  public void setOffset(double offset) {
    pubOffset.set(offset);
    this.offset = () -> offset;
  }

  public void setCurrentPoseAsZero() {
    motor.setEncoderPosition(Degrees.of(0));
  }

  public Command getDegreeSetter(double degrees) {
    // Returns the command that sets current degrees to turret

    return turret.setAngle(Degrees.of(degrees + offset.getAsDouble()))
        .until(() -> turret.getAngle().in(Degrees) < degrees + offset.getAsDouble() + 5
            && turret.getAngle().in(Degrees) > degrees + offset.getAsDouble() - 5);
  }

  public Command getDegreeSetter(DoubleSupplier degreesSupplier) {
    // Returns the command that sets current degrees to turret
    Double degrees = degreesSupplier.getAsDouble();
    return turret.setAngle(Degrees.of(degrees + offset.getAsDouble()))
        .until(() -> turret.getAngle().in(Degrees) < degrees + offset.getAsDouble() + 5
            && turret.getAngle().in(Degrees) > degrees + offset.getAsDouble() - 5);
  }

  public Angle getAngle() {
    return turret.getAngle();
  }

  public void setZeroCommand() {
    angleCommand.cancel();

    motor.stopClosedLoopController();
  }

  public void setSpeed(double dc) {
    dc *= -.3;
    motor.setDutyCycle(dc);
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
    CommandScheduler.getInstance().schedule(turret.sysId(Volts.of(4), Volts.of(0.5).per(Second), Seconds.of(8)));
  }

  @Override
  public void periodic() {
    turret.updateTelemetry();
    yamsRotation.set(turret.getAngle().in(Degree));

    if (isSim) {
      turretSimulation.setTurretDegrees(turret.getAngle().in(Degree));
      yamsAngleShowerPose.set(new Pose3d(robotPose.get().getTranslation().getX(),
          robotPose.get().getTranslation().getY(),
          1,
          new Rotation3d(0.0, 0.0,
              Rotation2d.fromDegrees(turret.getAngle().in(Degree)).plus(robotPose.get().getRotation()).getRadians())));
      // This method will be called once per scheduler run
    }

  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }
}