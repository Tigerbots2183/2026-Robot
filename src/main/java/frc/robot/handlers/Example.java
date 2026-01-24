// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Example extends SubsystemBase implements StateSubsystem {
  /** Creates a new Example. */
  public Example() {
    stateShower.set("IDLE");
  }

  private static Example m_Instance;
  private ExampleStates desiredState, currentState = ExampleStates.IDLE;

  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable stateTable = networkTable.getTable("RobotStates");
  private final StringPublisher stateShower = stateTable.getStringTopic("ExampleState").publish(); //TODO: Change with name


  public enum ExampleStates implements State {
    IDLE,
    BROKEN,
    MANUAL,
  }

  public void handleStateTransition(){
    switch (desiredState) {
      case IDLE:
        stateShower.set("IDLE");
        break;
      case BROKEN:
        stateShower.set("BROKEN");
        break;
      case MANUAL:
        stateShower.set("MANUAL");
        break;
      default:
        stateShower.set("UNKNOWN");
        break;
    }
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
      desiredState = (ExampleStates) state;
      handleStateTransition();
    }
  }

  public static Example getInstance(){
    if(m_Instance == null){
      m_Instance = new Example();
    }
    return m_Instance;
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }
}
