// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Console;
import java.io.File;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.VecBuilder;
import gg.questnav.questnav.PoseFrame;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import gg.questnav.questnav.QuestNav;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Touchboard.Touchboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class QuestNavSubsystem extends SubsystemBase {
    /** Creates a new QuestNavSubsystem. */
    private final CommandSwerveDrivetrain s_Drivetrain;

    QuestNav questNav = new QuestNav();

    // Assume this is the requested reset pose
    Pose3d robotPoseBlue = Constants.QuestNavConstants.initalPose3dBlue
            .transformBy(Constants.QuestNavConstants.ROBOT_TO_QUEST);
    Pose3d robotPoseRed = Constants.QuestNavConstants.initalPose3dRed
            .transformBy(Constants.QuestNavConstants.ROBOT_TO_QUEST);
    StructPublisher<Pose3d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("/QuestPose", Pose3d.struct).publish();
    StructPublisher<Pose3d> nonTranslatedPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("/QuestPoseNoTrans", Pose3d.struct).publish();

    private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
    private final NetworkTable visionTable = networkTable.getTable("Vision");
    private final NetworkTable tbTable = networkTable.getTable("touchboard");

    BooleanPublisher hasQuest = visionTable.getBooleanTopic("Quest").publish();
    StringSubscriber initalPose = tbTable.getStringTopic("initalPose").subscribe("BlueLeft");


    Boolean haveQuest = false;

    // Send the reset operation

    public QuestNavSubsystem(CommandSwerveDrivetrain swerveSubsystem) {
        this.s_Drivetrain = swerveSubsystem;

        Touchboard.bindActionButton("set", ()-> Commands.runOnce(()-> setPoseFromString(initalPose.get())).ignoringDisable(true));


        setPoseFromString(initalPose.get());
    }

    public void setPose(Pose3d position) {
        questNav.setPose(position.transformBy(Constants.QuestNavConstants.ROBOT_TO_QUEST));
    }

    public void setInitialPose() {
        if (DriverStation.getAlliance().isPresent()) {
            // If there is a driver station, check which one
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                questNav.setPose(robotPoseRed);
            } else {
                questNav.setPose(robotPoseBlue);
            }
        } else {
            questNav.setPose(robotPoseBlue);
        }
    }

    Pose2d BlueLeft = new Pose2d(4.401, 7.625, new Rotation2d());
    Pose2d BlueRight = new Pose2d(4.401, 0.479 , new Rotation2d());
    Pose2d RedLeft = FlippingUtil.flipFieldPose(BlueLeft);
    Pose2d RedRight = FlippingUtil.flipFieldPose(BlueRight);

    

    public void setPoseFromString(String where){
        if(where == "BlueLeft"){
            questNav.setPose(new Pose3d(BlueLeft));
        } else if (where == "BlueRight"){
            questNav.setPose(new Pose3d(BlueRight));
        
        }else if (where == "RedLeft"){
            questNav.setPose(new Pose3d(RedLeft));
        
        }else if (where == "RedRight"){
            questNav.setPose(new Pose3d(RedRight));
        
        }
    }

    public boolean haveQuest() {
        return questNav.isTracking();
    }

    public int questPercent() {
        if (questNav.getBatteryPercent().isPresent()) {
            return questNav.getBatteryPercent().getAsInt();
        }
        return 0;
    }

    Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
            0.02, // Trust down to 2cm in Y direction
            0.035 // Trust down to 2 degrees rotational
    );

    PoseFrame[] questFrames;
    PoseFrame questFrame;

    Pose3d currentQuestPose;
    double timestamp;
    Pose3d robotPose;

    boolean doRejectUpdate;

    @Override
    public void periodic() {

        questNav.commandPeriodic();

        if (questNav.getBatteryPercent().isPresent()) {
            SmartDashboard.putNumber("QuestPercent", questNav.getBatteryPercent().getAsInt());
        }

        if (questNav.isTracking()) {
            hasQuest.set(true);
            // Get the latest pose data frames from the Quest
            questFrames = questNav.getAllUnreadPoseFrames();

            if (questFrames.length > 0) {
                // Loop over the pose data frames and send them to the pose estimator
                questFrame = questFrames[questFrames.length - 1];
                // Get the pose of the Quest
                currentQuestPose = questFrame.questPose3d();
                // Get timestamp for when the data was sent
                timestamp = questFrame.dataTimestamp();
                // Transform by the mount pose to get your robot pose
                nonTranslatedPublisher.set(currentQuestPose);

                robotPose = currentQuestPose.transformBy(Constants.QuestNavConstants.ROBOT_TO_QUEST.inverse());
                posePublisher.set(robotPose);

                // You can put some sort of filtering here if you would like!
                // Add the measurement to our estimator
                s_Drivetrain.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
            }
        } else {
            hasQuest.set(false);
            setFromMt1("limelight-rsl");
            setFromMt1("limelight-quest");
        }
    }

    private void setFromMt1(String name) {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        doRejectUpdate = false;

        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > .7) {
                doRejectUpdate = true;
            }
            if (mt1.rawFiducials[0].distToCamera > 3) {
                doRejectUpdate = true;
            }
        }
        if (mt1.tagCount == 0) {
            doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
            s_Drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
            s_Drivetrain.addVisionMeasurement(
                    mt1.pose,
                    mt1.timestampSeconds);
        }
    }

    public static QuestNavSubsystem m_Instance;

    public static QuestNavSubsystem getInstance() {
        if (m_Instance == null) {
            m_Instance = new QuestNavSubsystem(TunerConstants.getInstance());
        }
        return m_Instance;
    }
}
