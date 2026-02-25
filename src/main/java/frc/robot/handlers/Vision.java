// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.s_Turret;
import frc.robot.subsystems.s_Vision;

public class Vision extends SubsystemBase implements StateSubsystem {
  /** Creates a new Example. */
  public Vision() {
    stateShower.set("IDLE");
  }

  private s_Vision vision = s_Vision.getInstance();
  private static Vision m_Instance;

  private VisionStates desiredState, currentState = VisionStates.PRESEEDING;

  private Trigger preSeedTrigger = RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
    this.setDesiredState(VisionStates.PRESEEDING);
  }));
  private Trigger SeededTriggerAuto = RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> {
    if (vision.seededOnce) {
      setDesiredState(VisionStates.READY);
    } else {
      setDesiredState(VisionStates.RESEEDING);
    }
  }));

  private Trigger SeededTriggerTele = RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
    if (vision.seededOnce) {
      setDesiredState(VisionStates.READY);
    } else {
      setDesiredState(VisionStates.RESEEDING);
    }
  }));

  // private Trigger reseedTrigger = new Trigger(()->
  // vision.reseed).onTrue(Commands.runOnce(()->{
  // setDesiredState(VisionStates.READY);
  // }));

  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable stateTable = networkTable.getTable("RobotStates");
  private final StringPublisher stateShower = stateTable.getStringTopic("VisionState").publish(); // TODO: Change with
                                                                                                  // name

  public enum VisionStates implements State {
    READY,
    PRESEEDING,
    RESEEDING,
    NONE,
  }

  public void handleStateTransition() {
    switch (desiredState) {
      case READY:
        stateShower.set("READY");
        break;
      case PRESEEDING:
        stateShower.set("PRESEEDING");
        break;
      case RESEEDING:
        stateShower.set("RESEEDING");
        break;
      case NONE:
        stateShower.set("NONE");
      default:
        stateShower.set("UNKNOWN");
        break;
    }
    currentState = desiredState;

  }

  public void update() {
    switch (currentState) {
      case READY:

        break;
      case PRESEEDING:
        vision.preseedFromMt1();
        break;
      case RESEEDING:
        if (vision.seed()) {
          this.setDesiredState(VisionStates.READY);
        }
        break;
      case NONE:
      default:
        break;
    }
  }

  public void setDesiredState(State state) {
    if (this.desiredState != state) {
      desiredState = (VisionStates) state;
      handleStateTransition();
    }
  }

  public static Vision getInstance() {
    if (m_Instance == null) {
      m_Instance = new Vision();
    }
    return m_Instance;
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }
}
