// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.s_Spindex;

public class Spindex extends SubsystemBase implements StateSubsystem {
  /** Creates a new Spindex. */
  public Spindex() {
    stateShower.set("IDLE");
  }

  private static Spindex m_Instance;
  private SpindexStates desiredState, currentState = SpindexStates.IDLE;

  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable stateTable = networkTable.getTable("RobotStates");
  private final StringPublisher stateShower = stateTable.getStringTopic("SpindexState").publish(); //TODO: Change with name

  private s_Spindex Spindex = s_Spindex.getInstance();

  public enum SpindexStates implements State {
    IDLE,
    BROKEN,
    FEEDING,
    MANUAL,
    REVERSE,
    STIRRING,
  }

  public void handleStateTransition(){
    switch (desiredState) {
      case IDLE:
        Spindex.setVoltage(0);
        stateShower.set("IDLE");
        break;
      case BROKEN:
        stateShower.set("BROKEN");
        break;
      case MANUAL:
        stateShower.set("MANUAL");
        break;
      case FEEDING:
        stateShower.set("FEEDING");
        Spindex.setVoltage(11);
        break;
      case STIRRING:
        stateShower.set("STIRRING");
        Spindex.setVoltage(4);
        break;
      case REVERSE: 
        stateShower.set("REVERSE");
        Spindex.setVoltage(-11);
      default:
        stateShower.set("UNKNOWN");
        break;
    }
    currentState = desiredState;

  }

  public void update(){
    switch (currentState) {
      case IDLE:
        break;
      case BROKEN:
        break;
      case MANUAL:
        break;
      default:
        break;
    }
  }

  
  public void setDesiredState(State state) {
    if (this.desiredState != state) {
      desiredState = (SpindexStates) state;
      handleStateTransition();
    }
  }

  public static Spindex getInstance(){
    if(m_Instance == null){
      m_Instance = new Spindex();
    }
    return m_Instance;
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }
}
