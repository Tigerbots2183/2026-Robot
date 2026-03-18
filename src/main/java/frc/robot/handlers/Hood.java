
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import java.util.function.Supplier;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.s_Hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meter;

import frc.robot.subsystems.Touchboard.Touchboard;

public class Hood extends SubsystemBase implements StateSubsystem {
  /** Creates a new Hood. */
  private static Hood m_instance;
  private HoodStates desiredState, currentState = HoodStates.TRACKING;
  private s_Hood hood = s_Hood.getInstance();

  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable stateTable = networkTable.getTable("RobotStates");
  private final NetworkTable touchboardTable = networkTable.getTable("touchboard");
  private final NetworkTable driveStateTable = networkTable.getTable("DriveState");

  private final StructPublisher<Pose2d> trGoalPose = driveStateTable.getStructTopic("goalPoseHood", Pose2d.struct).publish();

  private DoubleSubscriber angle = touchboardTable.getDoubleTopic("tbAngle").subscribe(0);
  // private final NetworkTable turretTable =
  // networkTable.getTable("TurretState");

  // private DoublePublisher hoodManualAngle =
  // turretTable.getDoubleTopic("ManualHoodAngle").publish();
  // private final NetworkTable hoodStateTable =
  // networkTable.getTable("HoodState");
  private final StringPublisher stateShower = stateTable.getStringTopic("HoodState").publish();

  private CommandSwerveDrivetrain s_Swerve = TunerConstants.getInstance();

  private Supplier<Pose2d> robotPoseSupplier = () -> s_Swerve.getState().Pose;

  private Supplier<ChassisSpeeds> chassisSpeedSupplier = ()-> ChassisSpeeds.fromRobotRelativeSpeeds(s_Swerve.getState().Speeds, robotPoseSupplier.get().getRotation());

  private final Supplier<Pose2d> goalPosition = () -> Turret.getInstance().getGoal();

  private final DoublePublisher goalDistance = driveStateTable.getDoubleTopic("GoalDistance").publish();

  private String alliance = "";

  public double findSpeedModifier(double speed) {
    switch (alliance) {
      case "":
        return -1.08 * speed;
      case "blue":
        return -1.08 * speed;
      case "red":
        return 1.08 * speed;
    }
    return 1.08 * speed;

  }

  public Hood() {
    hood.setDegreeCommand();
    this.setDesiredState(desiredState);

    
    RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop()).onTrue(Commands.runOnce(() -> {
      if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
          alliance = "red";

        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
          alliance = "blue";
        }
      }

    }));
  }

  public static Hood getInstance() {
    if (m_instance == null) {
      m_instance = new Hood();
    }
    return m_instance;
  }
  Pose2d currentGoalPosition;
  Pose2d translatedGoalPose;

  double currentDegree;
  
  Pose2d translatedTurretPose;
  double dist;
  ChassisSpeeds speeds;

  public void update() {

    switch (currentState) {
      case IDLE:
        hood.stop();
        break;
      case BROKEN:
        hood.stop();
        break;
      case TRACKING:
        // hood.setDegrees(45);
        speeds = chassisSpeedSupplier.get();

        currentGoalPosition = goalPosition.get();


        translatedGoalPose = currentGoalPosition.transformBy(new Transform2d(findSpeedModifier(speeds.vxMetersPerSecond),
                findSpeedModifier(speeds.vyMetersPerSecond), new Rotation2d()));

        trGoalPose.set(translatedGoalPose);

        translatedTurretPose = robotPoseSupplier.get().transformBy(new Transform2d(0.196, 0.0, new Rotation2d()));

        dist = Meter.of(Math.sqrt(Math.pow((translatedTurretPose.getX() - translatedGoalPose.getX()), 2)
            + Math.pow((translatedTurretPose.getY() - translatedGoalPose.getY()), 2))).in(Feet);
        goalDistance.set(dist);



        if (dist < 9.5) {
          // hood.setDegrees(0.0573934x^{3}-0.797824x^{2}+4.95428x;
          // y= 4.40995x-7.32742
          currentDegree = 4.40995 * dist -7.32742;
        } else if (dist < 11.75){
          // y=13.69863x-113.0137
          currentDegree = 13.69863 * dist -113.0137;
        } else if (dist < 14.25) {
          // y=7.40741x-62.44444
          currentDegree = 7.40741*dist-62.444449;
        } else if (dist < 16){
          currentDegree = 3.31126 * dist -21.31788;
        } else if (dist < 17.5){
          currentDegree = 6.20155 * dist -79.02713;
        } else if (dist < 20){
          currentDegree = 7.43802 * dist -117.22727;
        }
        
        hood.setDegrees(currentDegree);

        break;
      case MANUAL:
      
        currentGoalPosition = goalPosition.get();
        translatedTurretPose = robotPoseSupplier.get().transformBy(new Transform2d(0.196, 0.0, new Rotation2d()));

        dist = Meter.of(Math.sqrt(Math.pow((translatedTurretPose.getX() - currentGoalPosition.getX()), 2)
            + Math.pow((translatedTurretPose.getY() - currentGoalPosition.getY()), 2))).in(Feet);
        goalDistance.set(dist);

        hood.setDegrees(angle.get());

        break;
      default:
        break;
    }
  }

  double currentManualDeg = 0;

  public void handleStateTransition() {
    switch (desiredState) {
      case WOAH:
        hood.setDegreeCommand();
        hood.setDegrees(30);
        break;
      case IDLE:
        hood.stop();

        stateShower.set("IDLE");
        break;
      case BROKEN:
        hood.stop();
        stateShower.set("BROKEN");
        break;
      case TRACKING:

        stateShower.set("TRACKING");
        hood.setDegreeCommand();
        break;
      case MANUAL:
        hood.setDegreeCommand();

        stateShower.set("MANUAL");
        break;
      case HOMING:
        CommandScheduler.getInstance().schedule(hood.getStallHome().andThen(Commands.runOnce(()->setDesiredState(HoodStates.TRACKING))));
        
      default:
        stateShower.set("UNKNOWN");
        break;
    }

    currentState = desiredState;
  }

  public void increaseDeg() {
    currentManualDeg += 2;
    hood.setOffset(currentManualDeg);
  }

  public void decreaseDeg() {
    currentManualDeg -= 2;
    hood.setOffset(currentManualDeg);
  }

  public void setDesiredState(State state) {
    if (this.desiredState != state) {
      desiredState = (HoodStates) state;
      handleStateTransition();
    }
  }

  public enum HoodStates implements State {
    IDLE,
    BROKEN,
    TRACKING,
    // INACCURATE,
    HOMING,
    WOAH,
    MANUAL,
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }
}
