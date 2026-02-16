// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.io.Console;
// import java.io.File;
// import java.util.Optional;
// import java.util.function.Supplier;

// import com.ctre.phoenix6.Utils;

// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.StructPublisher;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.VecBuilder;
// import gg.questnav.questnav.PoseFrame;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Transform2d;
// import gg.questnav.questnav.QuestNav;
// import frc.robot.RobotContainer;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class QuestNavSubsystem extends SubsystemBase {
//   /** Creates a new QuestNavSubsystem. */
//   CommandSwerveDrivetrain s_Drivetrain;

//   QuestNav questNav = new QuestNav();

//   // Assume this is the requested reset pose
//   Pose2d robotPoseBlue = Constants.QuestNavConstants.initalPose2dBlue.transformBy(Constants.QuestNavConstants.ROBOT_TO_QUEST);;
//   Pose2d robotPoseRed = Constants.QuestNavConstants.initalPose2dRed.transformBy(Constants.QuestNavConstants.ROBOT_TO_QUEST);;
//   StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("/QuestPose", Pose2d.struct).publish();

//   Boolean haveQuest = false; 

//   // Send the reset operation

//   public QuestNavSubsystem(CommandSwerveDrivetrain swerveSubsystem) {
//     this.s_Drivetrain = swerveSubsystem;
    
//     if(DriverStation.getAlliance().isPresent()){
//       if(DriverStation.getAlliance().get() == Alliance.Red){
//         questNav.setPose3d(robotPoseRed);
//       }
//       else{
//         questNav.setPose3d(robotPoseBlue);
//       }

//     } else{
//       questNav.setPose3d(robotPoseBlue);
//     }
//   }

//   public void setPose(Pose2d position){
//     questNav.setPose3d(position);
//   }

  
 
//   public void setInitialPose(){
//     if(DriverStation.getAlliance().isPresent()){
//       //If there is a driver station, check which one
//       if(DriverStation.getAlliance().get() == Alliance.Red){ 
//         questNav.setPose3d(robotPoseRed);
//       } else {
//         questNav.setPose3d(robotPoseBlue);
//       }
//     } else{
//       questNav.setPose3d( robotPoseBlue );
//     }
//   }

//   public boolean haveQuest(){
//     return questNav.isTracking();
//   }

//   public int questPercent(){
//     if(questNav.getBatteryPercent().isPresent()){
//       return questNav.getBatteryPercent().getAsInt();
//     }
//     return 0;
//   }
  
//   @Override
//   public void periodic() {

//     questNav.commandPeriodic();

//     Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
//         0.02, // Trust down to 2cm in X direction
//         0.02, // Trust down to 2cm in Y direction
//         0.035 // Trust down to 2 degrees rotational
//     );

//     if(questNav.getBatteryPercent().isPresent()){
//       SmartDashboard.putNumber("QuestPercent", questNav.getBatteryPercent().getAsInt());
//     }

//     if (questNav.isTracking()) {
//       // Get the latest pose data frames from the Quest
//       PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

//       // Loop over the pose data frames and send them to the pose estimator
//       for (PoseFrame questFrame : questFrames) {
//         // Get the pose of the Quest
//         Pose3d questPose = questFrame.questPose3d();
//         // Get timestamp for when the data was sent
//         double timestamp = questFrame.dataTimestamp();
//         // Transform by the mount pose to get your robot pose
//         Pose2d robotPose = questPose.transformBy(Constants.QuestNavConstants.ROBOT_TO_QUEST.inverse());
//         posePublisher.set(robotPose);

//         // You can put some sort of filtering here if you would like!
//         // Add the measurement to our estimator
        
//         s_Drivetrain.addVisionMeasurement(robotPose,timestamp, QUESTNAV_STD_DEVS);
//       }
//     } else {
//       System.out.println("No QUEST");
//     }
//   }
// }
