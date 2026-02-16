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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;

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

  private final PIDController robotXController = new PIDController(9, 0, 0);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController controller = new CommandXboxController(0);

  private DoubleSupplier setX;
  private DoubleSupplier setY;
  private DoubleSupplier setRot;

  final SwerveRequest.Idle idle = new SwerveRequest.Idle();

  private Command idleDrive = drivetrain.applyRequest(() -> idle);

  private Command defaultDrive = drivetrain.applyRequest(() -> drive
      .withVelocityX(setX.getAsDouble() * MaxSpeed)
      .withVelocityY(setY.getAsDouble() * MaxSpeed)
      .withRotationalRate(-rotStick.getAsDouble() * MaxAngularRate));

  public s_Drivetrain() {
    initialized = true;

    xStick = () -> controller.getLeftX();
    yStick = () -> controller.getLeftY();
    rotStick = () -> controller.getRightX();

    final var idle = new SwerveRequest.Idle();

    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    controller.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric).ignoringDisable(true));

    drivetrain.registerTelemetry(logger::telemeterize);

    drivetrain.setDefaultCommand(defaultDrive);
  }

  public static s_Drivetrain getInstance() {
    if (m_Instance == null) {
      m_Instance = new s_Drivetrain();
    }
    return m_Instance;
  }

  public void setController(){
    drivetrain.setDefaultCommand(defaultDrive);
  }

  public void setValuesController() {
    setY = () -> Math.copySign((xStick.getAsDouble() * xStick.getAsDouble()), (-xStick.getAsDouble()));
    setX = () -> Math.copySign((yStick.getAsDouble() * yStick.getAsDouble()), (-yStick.getAsDouble()));
    setRot = () -> -rotStick.getAsDouble() * MaxAngularRate;
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
    // This method will be called once per scheduler run
  }
}
