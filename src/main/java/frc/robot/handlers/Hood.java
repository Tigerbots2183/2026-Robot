
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.s_Hood;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meter;

import frc.robot.subsystems.Touchboard.Touchboard;

public class Hood extends SubsystemBase implements StateSubsystem {
  /** Creates a new Hood. */
  private static Hood m_instance;
  private HoodStates desiredState, currentState = HoodStates.MANUAL;
  private s_Hood hood = s_Hood.getInstance();

  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable stateTable = networkTable.getTable("RobotStates");
  private final NetworkTable touchboardTable = networkTable.getTable("touchboard");
  private final NetworkTable driveStateTable = networkTable.getTable("DriveState");

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

  private final Supplier<Pose2d> goalPosition = () -> Turret.getInstance().getGoal();

  private final DoublePublisher goalDistance = driveStateTable.getDoubleTopic("GoalDistance").publish();

  public Hood() {
    hood.setDegreeCommand();
    this.setDesiredState(desiredState);
  }

  public static Hood getInstance() {
    if (m_instance == null) {
      m_instance = new Hood();
    }
    return m_instance;
  }

  Pose2d currentGoalPosition;
  Pose2d translatedTurretPose;
  double dist;

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

        currentGoalPosition = goalPosition.get();
        translatedTurretPose = robotPoseSupplier.get().transformBy(new Transform2d(0.196, 0.0, new Rotation2d()));

        dist = Meter.of(Math.sqrt(Math.pow((translatedTurretPose.getX() - currentGoalPosition.getX()), 2)
            + Math.pow((translatedTurretPose.getY() - currentGoalPosition.getY()), 2))).in(Feet);
        goalDistance.set(dist);

        if (dist < 13 + 1.83333333333) {
          hood.setDegrees((dist * 1.43284) + 8.03284);

        }
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
      default:
        stateShower.set("UNKNOWN");
        break;
    }

    currentState = desiredState;
  }

  public void increaseDeg() {
    currentManualDeg += 2.5;
  }

  public void decreaseDeg() {
    currentManualDeg -= 2.5;

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
    MANUAL,
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }
}
