
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import java.util.function.Supplier;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.s_Hood;

public class Hood extends SubsystemBase implements StateSubsystem {
  /** Creates a new Hood. */
  private static Hood m_instance;
  private HoodStates desiredState, currentState = HoodStates.MANUAL;
  private s_Hood hood = s_Hood.getInstance();

  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable stateTable = networkTable.getTable("RobotStates");
  private final NetworkTable driveStateTable = networkTable.getTable("DriveState");
  // private final NetworkTable hoodStateTable =
  // networkTable.getTable("HoodState");
  private final StringPublisher stateShower = stateTable.getStringTopic("HoodState").publish();

  private CommandSwerveDrivetrain s_Swerve = TunerConstants.getInstance();
  private Supplier<Pose2d> robotPoseSupplier = () -> s_Swerve.getState().Pose;

  private final Supplier<Pose2d> goalPosition = ()-> Turret.getInstance().getGoal();

  private final DoublePublisher goalDistance = driveStateTable.getDoubleTopic("GoalDistance").publish();

  public Hood() {
    stateShower.set("IDLE");

  }

  public static Hood getInstance() {
    if (m_instance == null) {
      m_instance = new Hood();
    }
    return m_instance;
  }

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
        
        Pose2d currentGoalPosition = goalPosition.get();
        Pose2d robotPose = robotPoseSupplier.get();
        Pose2d translatedTurretPose = robotPose.transformBy(new Transform2d(0.196, 0.0, new Rotation2d()));

        double dist = Math.sqrt(Math.pow((translatedTurretPose.getX() - currentGoalPosition.getX()), 2)
            + Math.pow((translatedTurretPose.getY() - currentGoalPosition.getY()), 2));
        goalDistance.set(dist);

        hood.setDegrees(dist * 5);
        break;
      case MANUAL:



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
        break;
      case MANUAL:
        hood.setDegrees(currentManualDeg);
        stateShower.set("MANUAL");
        break;
      default:
        stateShower.set("UNKNOWN");
        break;
    }

    currentState = desiredState;
  }

  public void increaseDeg(){
    currentManualDeg++;
  }

  public void decreaseDeg(){
    currentManualDeg--;
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
