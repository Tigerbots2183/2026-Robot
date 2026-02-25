// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.s_Drivetrain;

public class Drivetrain extends SubsystemBase implements StateSubsystem {
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    stateShower.set("TELEOP");

    inTrenchTrigger.and(Drivetrain.leftPressTrigger).onTrue(Commands.runOnce(()-> this.setDesiredState(DrivetrainStates.TRENCH)));
    inTrenchTrigger.onFalse(Commands.runOnce(()-> this.setDesiredState(DrivetrainStates.TELEOP)));
    Drivetrain.leftPressTrigger.onFalse(Commands.runOnce(()-> this.setDesiredState(DrivetrainStates.TELEOP)));

  }

  private static Drivetrain m_Instance;
  private DrivetrainStates desiredState, currentState = DrivetrainStates.TELEOP;

  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable stateTable = networkTable.getTable("RobotStates");
  private final StringPublisher stateShower = stateTable.getStringTopic("DriveState").publish();

  private s_Drivetrain Drivetrain = s_Drivetrain.getInstance();

  private CommandSwerveDrivetrain drive = TunerConstants.getInstance();

  private Trigger inTrenchTrigger = new Trigger(this::inTrenchZone);

  public enum DrivetrainStates implements State {
    IDLE,
    TELEOP,
    TRENCH,
    PATHING,
  }

  public void handleStateTransition() {
    switch (desiredState) {
      case IDLE:
        Drivetrain.idleOut();
        stateShower.set("IDLE");
        break;
      case TELEOP:
        Drivetrain.setController();
        stateShower.set("TELEOP");
        break;
      case TRENCH:
        Drivetrain.setTrenchLock();        
        stateShower.set("TRENCH");
        break;
      default:
        stateShower.set("UNKNOWN");
        break;
    }
    currentState = desiredState;

  }

  public void update() {
    switch (currentState) {
      case IDLE:
        break;
      case TELEOP:
        Drivetrain.setValuesController();
        break;
      case TRENCH:
        Drivetrain.setValuesTrench();
        break;
      default:
        break;
    }
  }

  public void setDesiredState(State state) {
    if (this.desiredState != state) {
      desiredState = (DrivetrainStates) state;
      handleStateTransition();
    }
  }

  private boolean inTrenchZone() {
    Pose2d robotPose = drive.getState().Pose;
    for (Translation2d[] zone : FieldConstants.TRENCH_ZONES) {
      if (robotPose.getX() >= zone[0].getX()
          && robotPose.getX() <= zone[1].getX()
          && robotPose.getY() >= zone[0].getY()
          && robotPose.getY() <= zone[1].getY()) {
        return true;
      }
    }
    return false;
  }

  public static Drivetrain getInstance() {
    if (m_Instance == null) {
      m_Instance = new Drivetrain();
    }
    return m_Instance;
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }
}
