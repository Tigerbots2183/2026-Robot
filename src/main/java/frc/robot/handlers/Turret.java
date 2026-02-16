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
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.s_Turret;
import frc.robot.subsystems.Touchboard.JukeboxUtil;
import edu.wpi.first.wpilibj2.command.*;

public class Turret extends SubsystemBase implements StateSubsystem {
  /** Creates a new TurretTracker. */

  private TurretStates desiredState, currentState = TurretStates.TRACKING;

  private s_Turret turret = s_Turret.getInstance();
  private static Turret m_Instance;

  private CommandSwerveDrivetrain s_swerve = TunerConstants.getInstance();
  private Supplier<Pose2d> robotPoseSupplier = () -> s_swerve.getState().Pose;

  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable driveStateTable = networkTable.getTable("DriveState/TurretTurntable");
  private final NetworkTable stateTable = networkTable.getTable("RobotStates");


  // private final StructPublisher<Pose2d> turretPose = driveStateTable.getStructTopic("turretPose", Pose2d.struct).publish();
  private final StructPublisher<Pose2d> goalPose = driveStateTable.getStructTopic("goalPose", Pose2d.struct).publish();
  private final DoublePublisher storedRotation = driveStateTable.getDoubleTopic("storedRotation").publish();

  private final StringPublisher stateShower = stateTable.getStringTopic("TurretState").publish();
 
  private boolean unwindCommandBound = false;

  private Pose2d goalPosition = new Pose2d(4.620419,4.034631, new Rotation2d());



  private double previousRotation = 0.0;
  private double currentRotation = 0.0;

  public Turret() {


    goalPose.set(goalPosition);
    stateShower.set("TRACKING");
  }

  public static Turret getInstance() {
    if (m_Instance == null) {
      m_Instance = new Turret();
    }
    return m_Instance;
  }

  public void setGoal(Supplier<Pose2d> goalPosition){
     this.goalPosition = goalPosition.get();
    goalPose.set(goalPosition.get());
  }

  public Pose2d getGoal(){
    return this.goalPosition;
  }

  public void update() {
    if(turret.inaccurate.get() && currentState == TurretStates.TRACKING){
      setDesiredState(TurretStates.INACCURATE);
    }else if(!turret.inaccurate.get()&& currentState == TurretStates.INACCURATE){
      setDesiredState(TurretStates.TRACKING);
    }
    
    switch (currentState) {
      case IDLE:
        break;
      case BROKEN:
        break;
      case INACCURATE:
      case TRACKING:
        Pose2d robotPose = robotPoseSupplier.get();

        Pose2d translatedTurretPose = robotPose.transformBy(new Transform2d(0.196, 0.0, new Rotation2d()));


        Rotation2d toGoal = Rotation2d.fromRadians(Math.atan2(goalPosition.getY() - translatedTurretPose.getY(),
            goalPosition.getX() - translatedTurretPose.getX()));

        Rotation2d robotRealtiveRotation = Rotation2d
            .fromRadians(toGoal.getRadians()- robotPose.getRotation().getRadians());
            
        double currentRotationChange = (robotRealtiveRotation.getDegrees() - previousRotation);
        

        if(currentRotationChange > 180){
          currentRotationChange = currentRotationChange - 360;
        } else if (currentRotationChange < -180){
          currentRotationChange = currentRotationChange + 360;
        }

        currentRotation += currentRotationChange;

        if(currentRotation > 1040){

          currentRotation = (currentRotation % 360) -360;

          this.setDesiredState(TurretStates.UNWINDING);

          return;
        } else if (currentRotation < -1040){
          currentRotation = (currentRotation % 360) + 360;

          this.setDesiredState(TurretStates.UNWINDING);

          return;
        }

        turret.setDegrees(currentRotation);


        storedRotation.set(currentRotation);

        previousRotation = robotRealtiveRotation.getDegrees();
        break;
      case UNWINDING:
        if(!unwindCommandBound){
          unwindCommandBound = true;
          
          Command turretResetter = turret.getDegreeSetter(currentRotation).andThen(new InstantCommand(()->{
            unwindCommandBound = false;
            setDesiredState(TurretStates.TRACKING);
            currentRotation = currentRotation % 360;
          }));

          CommandScheduler.getInstance().schedule(turretResetter);
        }
        
        break;
      default:
        break;
    }

  }

  public void setDesiredState(State state) {
    if (this.desiredState != state) {
      desiredState = (TurretStates) state;
      handleStateTransition();
    }
  }

  public void handleStateTransition() {

    if(currentState == TurretStates.TRACKING && desiredState == TurretStates.INACCURATE){
      stateShower.set("INACCURATEstateSkip");

      currentState = desiredState;

      return;
    }else if (currentState == TurretStates.INACCURATE && desiredState == TurretStates.TRACKING){
      stateShower.set("TRACKINGstateSkip");

      currentState = desiredState;

      return;
    }

    switch (desiredState) {
      case IDLE:
        stateShower.set("IDLE");
        turret.stop();
        break;
      case BROKEN:
        stateShower.set("BROKEN");

        turret.stop();
        break;
      case TRACKING:
        stateShower.set("TRACKING");
        turret.stop();
        unwindCommandBound = false;
        goalPose.set(goalPosition);
        break;
      case INACCURATE:
        stateShower.set("INACCURATE");
        turret.stop();
        unwindCommandBound = false;
        goalPose.set(goalPosition);
        break;
      case UNWINDING:
        stateShower.set("UNWINDING");

        turret.stop();
        break;
      case SYSID:
        stateShower.set("SYSID");
        
        JukeboxUtil.getInstance().mOrchestra.stop();

        turret.runSYSID();
        break;
      default:
        stateShower.set("UNKNOWN");
        break;
    }

    currentState = desiredState;
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }

  public enum TurretStates implements State {
    IDLE,
    BROKEN,
    TRACKING,
    INACCURATE,
    MANUAL,
    UNWINDING,
    SYSID,
  }
}