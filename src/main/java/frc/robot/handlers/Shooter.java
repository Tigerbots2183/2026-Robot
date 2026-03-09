// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meter;

import java.util.function.Supplier;

import com.fasterxml.jackson.databind.ser.BeanSerializer;

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
import frc.robot.subsystems.s_Shooter;
import frc.robot.subsystems.Touchboard.Touchboard;

public class Shooter extends SubsystemBase implements StateSubsystem {
  /** Creates a new Shooter. */

  public Shooter() {
    stateShower.set("SHOOTING");
    // manualRpm = new NumberComponent("tbRpm");
    // manualIndex = new NumberComponent("tbIndex");

  }

  private static Shooter m_Instance;
  private ShooterStates desiredState, currentState = ShooterStates.IDLE;

  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable stateTable = networkTable.getTable("RobotStates");
  private final StringPublisher stateShower = stateTable.getStringTopic("ShooterState").publish();

  private final Supplier<Pose2d> goalPosition = () -> Turret.getInstance().getGoal();

  private CommandSwerveDrivetrain s_Swerve = TunerConstants.getInstance();
  private Supplier<Pose2d> robotPoseSupplier = () -> s_Swerve.getState().Pose;

  private final NetworkTable turretTable = networkTable.getTable("TurretState");
  private final NetworkTable touchboardTable = networkTable.getTable("touchboard");

  private final DoublePublisher flywheelRpm = turretTable.getDoubleTopic("Flywheel Rpm").publish();
  private final DoublePublisher timePublish = turretTable.getDoubleTopic("Time out").publish();

  private DoubleSubscriber rpmTB = touchboardTable.getDoubleTopic("tbRpm").subscribe(0);

  private s_Shooter Shooter = s_Shooter.getInstance();

  public enum ShooterStates implements State {
    IDLE,
    BROKEN,
    SHOOTING,
    MANUAL,
    REVVING,
    REVERSE,
    TRENCH,
    WOAH,
  }

  Pose2d currentGoalPosition;
  Pose2d translatedTurretPose;
  Double dist;

  double timeout = 0.0;

  double revRpm;
  public void handleStateTransition() {
    switch (desiredState) {
      case WOAH: 
        Shooter.setRPM(3000);
        Shooter.setIndexVolts(11);
        break;

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
        Shooter.setIndexVolts(11);
        Shooter.setShooterCommand();

        currentGoalPosition = goalPosition.get();
        translatedTurretPose = robotPoseSupplier.get().transformBy(new Transform2d(0.196, 0.0, new Rotation2d()));

        dist = Meter.of(Math.sqrt(Math.pow((translatedTurretPose.getX() - currentGoalPosition.getX()), 2)
            + Math.pow((translatedTurretPose.getY() - currentGoalPosition.getY()), 2))).in(Feet);

        Shooter.setRPM(rpmTB.get());

        break;
      case SHOOTING:
        stateShower.set("SHOOTING");
        Shooter.setIndexVolts(11);
        Shooter.setShooterCommand();

        currentGoalPosition = goalPosition.get();
        translatedTurretPose = robotPoseSupplier.get().transformBy(new Transform2d(0.196, 0.0, new Rotation2d()));

        dist = Meter.of(Math.sqrt(Math.pow((translatedTurretPose.getX() - currentGoalPosition.getX()), 2)
            + Math.pow((translatedTurretPose.getY() - currentGoalPosition.getY()), 2))).in(Feet);

        if (dist < 8.5) {
          Shooter.setRPM(2000);

        } else if (dist < 11.5){
          Shooter.setRPM(2075);

        }else{
         Shooter.setRPM(2200);

        }

        break;

      case REVVING:
        stateShower.set("REVVING");
        // Shooter.setIndexSpeed(11);
        timeout = 0.0;
        Shooter.setShooterCommand();

        currentGoalPosition = goalPosition.get();
        translatedTurretPose = robotPoseSupplier.get().transformBy(new Transform2d(0.196, 0.0, new Rotation2d()));

        dist = Meter.of(Math.sqrt(Math.pow((translatedTurretPose.getX() - currentGoalPosition.getX()), 2)
            + Math.pow((translatedTurretPose.getY() - currentGoalPosition.getY()), 2))).in(Feet);

        if (dist < 8.5) {
          Shooter.setRPM(2000);

        } else if (dist < 11.5){
          Shooter.setRPM(2075);

        }else{
         Shooter.setRPM(2200);

        }
        
        break;

      case REVERSE:
        Shooter.setIndexSpeed(-1);
        Shooter.setShooterCommand();
        Shooter.setRPM(-500);
        break;
      case TRENCH:
        stateShower.set("TRENCH");
        Shooter.setIndexVolts(11);
        Shooter.setShooterCommand();
        Shooter.setRPM(2000);
        break;

      default:
        stateShower.set("UNKNOWN");
        break;
    }

    currentState = desiredState;
  }


  public void update() {
    flywheelRpm.set(Shooter.getVelocity());
    switch (currentState) {
      case IDLE:
        break;
      case BROKEN:
        break;
      case MANUAL:
        break;
      case REVVING:
        timeout += 0.3;
       timePublish.set(timeout);
        if(timeout > 5){
        Shooter.setIndexSpeed(11);

        }
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

  public static Shooter getInstance() {
    if (m_Instance == null) {
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
