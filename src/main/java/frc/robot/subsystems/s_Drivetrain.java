// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.FieldConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.driveConstants.MaxSpeed;
import static frc.robot.Constants.driveConstants.MaxAngularRate;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class s_Drivetrain extends SubsystemBase implements CheckableSubsystem {
  /** Creates a new s_Example. */
  public static s_Drivetrain m_Instance;
  private CommandSwerveDrivetrain drivetrain = TunerConstants.getInstance();

  private DoubleSupplier xStick;
  private DoubleSupplier yStick;
  private DoubleSupplier rotStick;

  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable driveStateTable = networkTable.getTable("DriveState/CommandTrain");

  private DoublePublisher PIDY = driveStateTable.getDoubleTopic("CALCY").publish();
  private DoublePublisher TrenchY = driveStateTable.getDoubleTopic("TrenchY").publish();
  private DoublePublisher RobotY = driveStateTable.getDoubleTopic("RobotY").publish();


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.FieldCentricFacingAngle trenchDriveRequest = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withHeadingPID(12, 0, 0);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController controller = new CommandXboxController(0);
  public Trigger leftPressTrigger = controller.leftStick();

  private DoubleSupplier setX;
  private DoubleSupplier setY;
  private DoubleSupplier calcY;
  private DoubleSupplier setRot;
  private Rotation2d closestRot;

  private Supplier<Pose2d> robotPose = () -> drivetrain.getState().Pose;

  final SwerveRequest.Idle idle = new SwerveRequest.Idle();

  private Command idleDrive = drivetrain.applyRequest(() -> idle);

  private Command defaultDrive = drivetrain.applyRequest(() -> drive
      .withVelocityX(setX.getAsDouble() * MaxSpeed)
      .withVelocityY(setY.getAsDouble() * MaxSpeed)
      .withRotationalRate(-rotStick.getAsDouble() * MaxAngularRate));

  private Command trenchDrive = drivetrain.applyRequest(() -> trenchDriveRequest
      .withVelocityX(setX.getAsDouble() * MaxSpeed)
      .withVelocityY(calcY.getAsDouble())
      .withTargetDirection(closestRot));

  private PIDController trenchPIDY = new PIDController(12, 0, 0);

  public s_Drivetrain() {
    initialized = true;

    xStick = () -> controller.getLeftX();
    yStick = () -> controller.getLeftY();

    if(RobotBase.isReal()){
      rotStick = () -> controller.getRightX();
    }else{
      rotStick = () -> controller.getRawAxis(2);
    }

    final var idle = new SwerveRequest.Idle();

    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    controller.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).ignoringDisable(true));
    controller.start().onTrue(Commands.runOnce(()->QuestNavSubsystem.getInstance().setPose(new Pose3d(new Pose2d(drivetrain.getState().Pose.getTranslation(), Rotation2d.fromDegrees(180))) )).ignoringDisable(true));

    drivetrain.registerTelemetry(logger::telemeterize);

    drivetrain.setDefaultCommand(defaultDrive);
  }

  public static s_Drivetrain getInstance() {
    if (m_Instance == null) {
      m_Instance = new s_Drivetrain();
    }
    return m_Instance;
  }

  public void setController() {
    this.calcY = () -> 0;

    trenchDrive.cancel();
    drivetrain.removeDefaultCommand();

    drivetrain.setDefaultCommand(defaultDrive);
  }

  public void setTrenchLock() {
    this.setY = () -> 0;

    defaultDrive.cancel();
    drivetrain.removeDefaultCommand();

    drivetrain.setDefaultCommand(trenchDrive);
  }

  public void setValuesController() {
    setY = () -> Math.copySign((xStick.getAsDouble() * xStick.getAsDouble()), (-xStick.getAsDouble()));
    setX = () -> Math.copySign((yStick.getAsDouble() * yStick.getAsDouble()), (-yStick.getAsDouble()));
    setRot = () -> -rotStick.getAsDouble() * MaxAngularRate;
  }

  public void setValuesTrench() {
    setX = () -> Math.copySign((yStick.getAsDouble() * yStick.getAsDouble()), (-yStick.getAsDouble()));
    // setRot = () -> -rotStick.getAsDouble() * MaxAngularRate;
    if (robotPose.get().getRotation().getDegrees() > -90 && robotPose.get().getRotation().getDegrees() < 90) {
      closestRot = Rotation2d.fromDegrees(180);

    } else {
      closestRot = Rotation2d.fromDegrees(0);
    }

    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        calcY = () -> -trenchPIDY.calculate(robotPose.get().getY(), getTrenchY().in(Meters));

      } else {
        calcY = () -> trenchPIDY.calculate(robotPose.get().getY(), getTrenchY().in(Meters));

      }
    } else {
      calcY = () -> trenchPIDY.calculate(robotPose.get().getY(), getTrenchY().in(Meters));

    }

    PIDY.set(calcY.getAsDouble());
    TrenchY.set(getTrenchY().in(Meters));
    RobotY.set(robotPose.get().getY());
  }

  private Distance getTrenchY() {
    Pose2d robotPose = this.robotPose.get();
    if (robotPose.getMeasureY().gte(FieldConstants.FIELD_WIDTH.div(2))) {
      return FieldConstants.FIELD_WIDTH.minus(FieldConstants.TRENCH_CENTER);
    }
    return FieldConstants.TRENCH_CENTER;
  }

  public void idleOut() {
    drivetrain.setDefaultCommand(idleDrive);
  }

  public boolean initialized = false;

  public boolean checkSubsystem() {
    return getInitialized();
  }

  public boolean getInitialized() {
    return initialized;
  }

  public void stop() {

  }

  @Override
  public void periodic() {
    // LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-rsl");
    // drivetrain.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
    // This method will be called once per scheduler run
  }
}
