// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.networktables.DoublePublisher;
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
  private final DoublePublisher DoubleShower = stateTable.getDoubleTopic("AngleAdder").publish(); 
  private double angleAdder = 0;

  public enum IntakeStates implements State {
    IDLE,
    BROKEN,
    OUT,
    REVERSE,
    INTAKING,
    RAISING,
    MANUAL,
    HUMAN,
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
        intake.setDegrees(-100);

        intake.setSpeed(0);
        break;
      case INTAKING:
        stateShower.set("INTAKING");

        // intake.setDegrees(-118);
        intake.setDegrees(-100);
        intake.setSpeed(-1);
        // intake.setDegrees(-12.5);

        // intake.setDegrees(70);
        break;

      case RAISING:
        stateShower.set("RAISING");

        intake.setDegrees(-100);
        intake.setSpeed(-.5);

        angleAdder =  -100;
        break;
      case REVERSE:
        intake.setSpeed(-1);

      case HUMAN:
        stateShower.set("RAISING");
        intake.setDegrees(0);
        intake.setSpeed(-.75);

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
      case RAISING:
        intake.setDegrees(angleAdder);
        if(angleAdder < -20){
        angleAdder += 0.8;

        }
        DoubleShower.set(angleAdder);

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
