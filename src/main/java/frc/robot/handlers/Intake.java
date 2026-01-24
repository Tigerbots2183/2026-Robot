// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.s_Intake;

public class Intake extends SubsystemBase implements StateSubsystem {
  /** Creates a new Intake. */
  public Intake() {
    stateShower.set("IDLE");
  }

  private s_Intake intake = s_Intake.getInstance();

  private static Intake m_Instance;
  private IntakeStates desiredState, currentState = IntakeStates.IDLE;

  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable stateTable = networkTable.getTable("RobotStates");
  private final StringPublisher stateShower = stateTable.getStringTopic("IntakeState").publish(); 


  public enum IntakeStates implements State {
    IDLE,
    BROKEN,
    OUT,
    INTAKING,
    MANUAL,
  }

  public void handleStateTransition(){

    switch (desiredState) {
      case IDLE:
        stateShower.set("IDLE");
        intake.setDegrees(0);
        intake.setSpeed(0);

        break;
      case BROKEN:
        stateShower.set("BROKEN");
        break;
      case MANUAL:
        stateShower.set("MANUAL");
        break;
      case OUT:
        stateShower.set("OUT");
        intake.setDegrees(90);
        intake.setSpeed(0);
        break;
      case INTAKING:
        stateShower.set("INTAKING");
        intake.setDegrees(90);
        intake.setSpeed(1.0);
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
      desiredState = (IntakeStates) state;
      handleStateTransition();
    }
  }

  public static Intake getInstance(){
    if(m_Instance == null){
      m_Instance = new Intake();
    }
    return m_Instance;
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }
}
