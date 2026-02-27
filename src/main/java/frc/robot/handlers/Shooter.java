// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.s_Shooter;

public class Shooter extends SubsystemBase implements StateSubsystem {
  /** Creates a new Shooter. */
  public Shooter() {
    stateShower.set("SHOOTING");

  }

  private static Shooter m_Instance;
  private ShooterStates desiredState, currentState = ShooterStates.IDLE;

  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable stateTable = networkTable.getTable("RobotStates");
  private final StringPublisher stateShower = stateTable.getStringTopic("ShooterState").publish();

  private final NetworkTable turretTable = networkTable.getTable("TurretState");
  
  private final DoublePublisher flywheelRpm = turretTable.getDoubleTopic("Flywheel Rpm").publish();

  private s_Shooter Shooter = s_Shooter.getInstance();

  public enum ShooterStates implements State {
    IDLE,
    BROKEN,
    SHOOTING,
    MANUAL,
    REVVING,
  }

  public void handleStateTransition(){
    switch (desiredState) {
      case IDLE:
        stateShower.set("IDLE");
        Shooter.setIndexVolts(0);
        Shooter.setStopCommand();
        break;
      case BROKEN:
        stateShower.set("BROKEN");
        break;
      case MANUAL:
        stateShower.set("MANUAL");
        break;
      case SHOOTING:
        stateShower.set("SHOOTING");
        Shooter.setIndexSpeed(.65);
        Shooter.setShooterCommand();
        Shooter.setRPM(3370);

        break;
      case REVVING:
        stateShower.set("REVVING");
        Shooter.setIndexSpeed(0);
        Shooter.setShooterCommand();
        Shooter.setRPM(3370);

      default:
        stateShower.set("UNKNOWN");
        break;
    }
  }

  public void update(){
    flywheelRpm.set(Shooter.getVelocity());
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
      desiredState = (ShooterStates) state;
      handleStateTransition();
    }
  }

  public static Shooter getInstance(){
    if(m_Instance == null){
      m_Instance = new Shooter();
    }
    return m_Instance;
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }
}
