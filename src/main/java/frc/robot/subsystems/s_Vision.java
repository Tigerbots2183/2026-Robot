// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.BooleanArrayEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.handlers.Vision;

public class s_Vision extends SubsystemBase implements CheckableSubsystem {
  /** Creates a new s_Example. */
  public static s_Vision m_Instance;
  // public static QuestNavSubsystem Qnav = QuestNavSubsystem.getInstance();

  public s_Vision() {
    initialized = true;
    seeded.set(false);
  }

  public static s_Vision getInstance() {
    if (m_Instance == null) {
      m_Instance = new s_Vision();
    }
    return m_Instance;
  }

  public boolean initialized = false;
  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable visionTable = networkTable.getTable("Vision");
  private final BooleanPublisher seeded = visionTable.getBooleanTopic("seeded").publish();
  private final DoublePublisher timeSinceSeeded = visionTable.getDoubleTopic("timeSinceSeed").publish();
  private final StructPublisher<Pose2d> lastSeededPose = visionTable.getStructTopic("lastSeededPose", Pose2d.struct)
      .publish();
  private final boolean isSim = RobotBase.isSimulation();
  private double sinceSeeded = 0;

  public boolean seededOnce = false;

  public void preseedFromMt1() {
    if(isSim){
      // Qnav.setInitialPose();
      seededOnce = true;

    }

    LimelightHelpers.PoseEstimate rsl = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-rsl");

    // System.out.println(rsl.tagCount);
    if (rsl.tagCount >= 2) {
      seededOnce = true;
      lastSeededPose.set(rsl.pose);
      seeded.set(true);
      // Qnav.setPose(new Pose3d(rsl.pose));
    }

  } 

  

  public boolean reseed = false;

  public boolean seed() {
    reseed = false;

    if(RobotBase.isSimulation()){
      // Qnav.setInitialPose();
      reseed = true;
    }


    seeded.set(false);
    LimelightHelpers.PoseEstimate rsl = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-rsl");

    if (rsl.tagCount >= 2) {
      sinceSeeded = Timer.getTimestamp();
      lastSeededPose.set(rsl.pose);
      seeded.set(true);
      // Qnav.setPose(new Pose3d(rsl.pose));

      return true;
    }

    return false;
  }

  
  public boolean checkSubsystem() {
    return getInitialized();
  }

  public boolean getInitialized() {
    return initialized;
  }

  public void stop() {

  }

  @Override
  public void periodic() {
    timeSinceSeeded.set(Timer.getTimestamp() - sinceSeeded);
    // This method will be called once per scheduler run
  }
}
