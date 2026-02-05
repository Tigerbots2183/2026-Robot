// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IO;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretIO extends SubsystemBase {
  /** Creates a new turretIO. */
  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable driveStateTable = networkTable.getTable("DriveState");

  private final StructPublisher<Pose3d> centralPose = driveStateTable.getStructTopic("centerPose", Pose3d.struct).publish();
  private final StructArrayPublisher<Pose3d> finalComponentPoses = driveStateTable.getStructArrayTopic("componentPoses", Pose3d.struct).publish();

  private Rotation2d hoodRotation = Rotation2d.fromDegrees(0);
  private Rotation2d turretRotation = Rotation2d.fromDegrees(0);

  private final StructPublisher<Pose2d> zeroPose = driveStateTable.getStructTopic("ZeroPose", Pose2d.struct).publish();
  
  private final StructArrayPublisher<Pose3d> zeroedComponentPoses = driveStateTable.getStructArrayTopic("ZeroedPoses", Pose3d.struct).publish();


  private static TurretIO m_Instance;

  public TurretIO() {
 zeroPose.set(new Pose2d(0,0, Rotation2d.fromDegrees(0)));
    
    zeroedComponentPoses.set(new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d()});
  }

  public void setHoodDegrees(double hoodDegrees){
    Pose3d pretransform = new Pose3d(0.17145000,0,0.2116331, new Rotation3d());

    this.hoodRotation = Rotation2d.fromDegrees(hoodDegrees);

    double x = Math.sin(pretransform.getRotation().getZ()) *  0.08558931;
    double y = Math.cos(pretransform.getRotation().getZ()) *  0.08558931;

    centralPose.set(new Pose3d(new Translation3d( 0.17145000 - x, 0 + y, 0.2116331), new Rotation3d( 0.0,0.0, 0.0)).rotateAround(new Translation3d(0.17145000,0,0.2116331), new Rotation3d( 0.0,0.0, turretRotation.getRadians())));

    finalComponentPoses.set(new Pose3d[] {
      new Pose3d(0.17145000,0,0.2116331, new Rotation3d(0.0, 0.0,  turretRotation.getRadians()
      )),
      pretransform.rotateAround(new Translation3d( 0.17145000 - x, 0 + y, 0.2116331), new Rotation3d((hoodRotation.getRadians()* -1) + Math.PI/6,0.0, 0.0)).rotateAround(new Translation3d(0.17145000,0,0.2116331), new Rotation3d( 0.0,0.0, turretRotation.getRadians()))
    });
  }

  public void setTurretDegrees(double turretDegrees){
    Pose3d pretransform = new Pose3d(0.17145000,0,0.2116331, new Rotation3d());


    this.turretRotation = Rotation2d.fromDegrees(turretDegrees);

    double x = Math.sin(pretransform.getRotation().getZ()) *  0.08558931;
    double y = Math.cos(pretransform.getRotation().getZ()) *  0.08558931;

    centralPose.set(new Pose3d(new Translation3d( 0.17145000 - x, 0 + y, 0.2116331), new Rotation3d( 0.0,0.0, 0.0)).rotateAround(new Translation3d(0.17145000,0,0.2116331), new Rotation3d( 0.0,0.0, turretRotation.getRadians())));

    finalComponentPoses.set(new Pose3d[] {
      new Pose3d(0.17145000,0,0.2116331, new Rotation3d(0.0, 0.0,  turretRotation.getRadians()
      )),
      pretransform.rotateAround(new Translation3d( 0.17145000 - x, 0 + y, 0.2116331), new Rotation3d((hoodRotation.getRadians()* -1) + Math.PI/6,0.0, 0.0)).rotateAround(new Translation3d(0.17145000,0,0.2116331), new Rotation3d( 0.0,0.0, turretRotation.getRadians()))
    });
  }

  public static TurretIO getInstance(){
    if(m_Instance == null) {
      m_Instance = new TurretIO();
    }
    return m_Instance;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
