// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.CopyOnWriteArrayList;

import javax.xml.namespace.QName;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.handlers.Drivetrain;
import frc.robot.handlers.Shooter;
import frc.robot.handlers.Hood;
import frc.robot.handlers.Intake;
import frc.robot.handlers.Shooter;
import frc.robot.handlers.Spindex;
import frc.robot.handlers.Turret;
import frc.robot.handlers.Vision;
import frc.robot.handlers.Hood.HoodStates;
import frc.robot.handlers.Vision.VisionStates;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.s_Hood;
import frc.robot.subsystems.s_Intake;
import frc.robot.subsystems.s_Shooter;
import frc.robot.subsystems.s_Turret;
import frc.robot.subsystems.Touchboard.Touchboard;

public class RobotContainer {

    private Intake H_Intake = Intake.getInstance();
    public Spindex H_Spindex = Spindex.getInstance();
    private Drivetrain H_Drivetrain = Drivetrain.getInstance();
    private Shooter H_Shooter = Shooter.getInstance();

    // private Vision H_Vision = Vision.getInstance();
    private QuestNavSubsystem Qnav = QuestNavSubsystem.getInstance();
    private Turret H_Turret = Turret.getInstance();
    private Hood H_Hood = Hood.getInstance();

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController coPilot = new CommandXboxController(1);

    private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();

    private final NetworkTable tbTable = networkTable.getTable("touchboard");
    StringSubscriber initalPose = tbTable.getStringTopic("initalPose").subscribe("BlueLeft");
    StringSubscriber auto = tbTable.getStringTopic("auton").subscribe("");

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.getInstance();
    public Command currentAuto;

    public RobotContainer() {
        configureBindings();
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
            if (RobotBase.isSimulation()) {
                drivetrain.runOnce(drivetrain::seedFieldCentric).ignoringDisable(true);
            }
        }));
        currentAuto = new PathPlannerAuto("2 swipe corral side mid and corral");
        // RobotModeTriggers.autonomous().onTrue(Commands.runOnce(()->
        // Q_Nav.setInitialPose()));
    }

    private void configureBindings() {

        joystick.rightBumper().onTrue(Commands.runOnce(() -> H_Intake.setDesiredState(Intake.IntakeStates.INTAKING)));
        joystick.rightBumper().onFalse(Commands.runOnce(() -> H_Intake.setDesiredState(Intake.IntakeStates.OUT)));

        coPilot.pov(90).onTrue(Commands.runOnce(() -> H_Turret.decreaseDeg()));
        coPilot.pov(270).onTrue(Commands.runOnce(() -> H_Turret.increaseDeg()));

        coPilot.start().onTrue(Commands.runOnce(() -> Qnav.setPoseFromString(() -> initalPose.get())));

        joystick.pov(0).onTrue(Commands.runOnce(() -> H_Intake.setDesiredState(Intake.IntakeStates.IDLE)));

        coPilot.pov(180).onTrue(Commands.runOnce(() -> H_Hood.decreaseDeg()));
        coPilot.pov(0).onTrue(Commands.runOnce(() -> H_Hood.increaseDeg()));

        joystick.leftBumper().onFalse(Commands.runOnce(() -> {
            H_Spindex.setDesiredState(Spindex.SpindexStates.IDLE);
            H_Shooter.setDesiredState(Shooter.ShooterStates.IDLE);
            Commands.runOnce(() -> H_Intake.setDesiredState(Intake.IntakeStates.OUT));
        }));

        joystick.leftBumper().onTrue(Commands.runOnce(() -> {
            H_Intake.setDesiredState(Intake.IntakeStates.REVERSE);
            H_Spindex.setDesiredState(Spindex.SpindexStates.REVERSE);
            H_Shooter.setDesiredState(Shooter.ShooterStates.REVERSE);
        }));

        joystick.rightTrigger(.5).onTrue(Commands.runOnce(() -> {
            H_Spindex.setDesiredState(Spindex.SpindexStates.FEEDING);
            H_Shooter.setDesiredState(Shooter.ShooterStates.SHOOTING);
            H_Intake.setDesiredState(Intake.IntakeStates.RAISING);
        }));

        joystick.rightTrigger(.5).onFalse(Commands.runOnce(() -> {
            H_Shooter.setDesiredState(Shooter.ShooterStates.IDLE);
            H_Intake.setDesiredState(Intake.IntakeStates.OUT);
            H_Spindex.setDesiredState(Spindex.SpindexStates.IDLE);
        }));

        joystick.leftTrigger(.5)
                .onTrue(Commands.runOnce(() -> H_Shooter.setDesiredState(Shooter.ShooterStates.REVVING)));
        joystick.leftTrigger(.5).onFalse(Commands.runOnce(() -> H_Shooter.setDesiredState(Shooter.ShooterStates.IDLE)));

        coPilot.rightBumper().onTrue(Commands.runOnce(() -> {
            H_Spindex.setDesiredState(Spindex.SpindexStates.FEEDING);
            H_Shooter.setDesiredState(Shooter.ShooterStates.SHOOTING);
            H_Intake.setDesiredState(Intake.IntakeStates.INTAKING);
        }));

        // coPilot.rightBumper().onTrue(Commands.runOnce(() ->
        // H_Intake.setDesiredState(Intake.IntakeStates.RAISING)));

        coPilot.rightBumper().onFalse(Commands.runOnce(() -> {
            H_Shooter.setDesiredState(Shooter.ShooterStates.IDLE);
            H_Intake.setDesiredState(Intake.IntakeStates.OUT);
            H_Spindex.setDesiredState(Spindex.SpindexStates.IDLE);
        }));

        coPilot.leftBumper().onTrue(Commands.runOnce(() -> H_Shooter.setDesiredState(Shooter.ShooterStates.REVVING)));
        coPilot.leftBumper().onFalse(Commands.runOnce(() -> H_Shooter.setDesiredState(Shooter.ShooterStates.IDLE)));

        coPilot.y().onTrue(Commands.runOnce(() -> {
            H_Shooter.setDesiredState(Shooter.ShooterStates.MANUAL);
            H_Hood.setDesiredState(Hood.HoodStates.MANUAL);
            H_Spindex.setDesiredState(Spindex.SpindexStates.FEEDING);
            H_Intake.setDesiredState(Intake.IntakeStates.RAISING);
        }));

        coPilot.y().onFalse(Commands.runOnce(() -> {
            H_Spindex.setDesiredState(Spindex.SpindexStates.IDLE);
            H_Hood.setDesiredState(Hood.HoodStates.TRACKING);
            H_Shooter.setDesiredState(Shooter.ShooterStates.IDLE);
            H_Intake.setDesiredState(Intake.IntakeStates.RAISING);
        }));

        coPilot.x().onTrue(Commands.runOnce(() -> {
            H_Hood.setDesiredState(Hood.HoodStates.MANUAL);
            H_Shooter.setDesiredState(Shooter.ShooterStates.MANUAL);
        }));

        coPilot.x().onFalse(Commands.runOnce(() -> {
            H_Hood.setDesiredState(Hood.HoodStates.TRACKING);
            H_Shooter.setDesiredState(Shooter.ShooterStates.IDLE);
        }));

        coPilot.b().toggleOnTrue(Commands.run(() -> H_Turret.setDesiredState(Turret.TurretStates.IDLE))
                .finallyDo(() -> H_Turret.setDesiredState(Turret.TurretStates.TRACKING)));

        coPilot.rightStick().toggleOnTrue(Commands.run(() -> H_Turret.setDesiredState(Turret.TurretStates.ZEROING))
                .finallyDo(() -> H_Turret.setDesiredState(Turret.TurretStates.TRACKING)));
        // coPilot.b().toggleOnFalse(Commands.runOnce(()->
        // H_Turret.setDesiredState(Turret.TurretStates.TRACKING)));

        // coPilot.leftStick().onTrue(Commands.runOnce(() -> {
        // H_Intake.setDesiredState(Intake.IntakeStates.REVERSE);
        // H_Spindex.setDesiredState(Spindex.SpindexStates.REVERSE);
        // H_Shooter.setDesiredState(Shooter.ShooterStates.REVERSE);
        // }));

        // coPilot.rightTrigger(0.8).onTrue(Commands.runOnce(() -> {
        // Pose2d aimPose ;
        // if (DriverStation.getAlliance().isPresent()) {
        // if (DriverStation.getAlliance().get() == Alliance.Blue) {
        // aimPose = new Pose2d(0.5, 1.6, new Rotation2d());

        // } else {
        // aimPose = new Pose2d(16, 6, new Rotation2d());

        // }
        // } else{
        // aimPose = new Pose2d(0.5, 1.6, new Rotation2d());

        // }

        // H_Turret.setGoal(()->aimPose);
        // }));

        // coPilot.leftTrigger(0.8).onTrue(Commands.runOnce(() -> {
        // Pose2d aimPose;
        // if (DriverStation.getAlliance().isPresent()) {
        // if (DriverStation.getAlliance().get() == Alliance.Blue) {
        // aimPose = new Pose2d(0.5, 6, new Rotation2d());

        // } else {
        // aimPose = new Pose2d(16, 1.9, new Rotation2d());

        // }
        // } else{
        // aimPose = new Pose2d(0.5, 1.6, new Rotation2d());

        // }

        // H_Turret.setGoal(()->aimPose);

        // }));

        coPilot.a().onTrue(Commands.runOnce(() -> {
            Pose2d goalPose = new Pose2d(4.620419, 4.034631, new Rotation2d());

            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    H_Turret.setGoal(() -> FlippingUtil.flipFieldPose(goalPose));
                    return;
                }
            }

            H_Turret.setGoal(() -> goalPose);

        }));
        joystick.rightStick().onTrue(Commands.runOnce(() -> {
            s_Turret.getInstance().setDegrees(0);
            H_Hood.setDesiredState(Hood.HoodStates.IDLE);
            H_Turret.setDesiredState(Turret.TurretStates.IDLE);
        }));

        joystick.rightStick().onTrue(Commands.runOnce(() -> {
            H_Intake.setDesiredState(Intake.IntakeStates.IDLE);
            s_Hood.getInstance().setDegrees(0);
            s_Intake.getInstance().setDegrees(0);
        }));

        drivetrain.runOnce(drivetrain::seedFieldCentric).ignoringDisable(true);

        // jukebox.addTalon(drivetrain.getModule(0).getDriveMotor());
        // jukebox.addTalon(drivetrain.getModule(1).getDriveMotor());
        // jukebox.addTalon(drivetrain.getModule(2).getDriveMotor());
        // jukebox.addTalon(drivetrain.getModule(3).getDriveMotor());

        // jukebox.addTalon(drivetrain.getModule(0).getSteerMotor());
        // jukebox.addTalon(drivetrain.getModule(1).getSteerMotor());
        // jukebox.addTalon(drivetrain.getModule(2).getSteerMotor());
        // jukebox.addTalon(drivetrain.getModule(3).getSteerMotor());

        // jukebox.addTalon(s_Intake.getInstance().getLeftPivotTalonFX());
        // jukebox.addTalon(s_Intake.getInstance().getRightPivotTalonFx());

        //

        NamedCommands.registerCommand("intake",
                Commands.runOnce(() -> H_Intake.setDesiredState(Intake.IntakeStates.INTAKING)));
        NamedCommands.registerCommand("sintake",
                Commands.runOnce(() -> H_Intake.setDesiredState(Intake.IntakeStates.OUT)));
        NamedCommands.registerCommand("revshoot",
                Commands.runOnce(() -> H_Shooter.setDesiredState(Shooter.ShooterStates.REVVING)));
        NamedCommands.registerCommand("sethood",
                Commands.runOnce(() -> H_Hood.setDesiredState(Hood.HoodStates.TRACKING)));
        NamedCommands.registerCommand("intakeup",
                Commands.runOnce(() -> H_Intake.setDesiredState(Intake.IntakeStates.RAISING)));
        // NamedCommands.registerCommand("trackturret", Commands.runOnce(() ->
        // H_Intake.setDesiredState(Turret.TurretStates.TRACKING)));
        NamedCommands.registerCommand("shoot",
                Commands.runOnce(() -> H_Shooter.setDesiredState(Shooter.ShooterStates.SHOOTING)));
        NamedCommands.registerCommand("spindex",
                Commands.runOnce(() -> H_Spindex.setDesiredState(Spindex.SpindexStates.FEEDING)));
        NamedCommands.registerCommand("stopshoot",
                Commands.runOnce(() -> H_Shooter.setDesiredState(Shooter.ShooterStates.IDLE)));
        NamedCommands.registerCommand("Zero", Commands.runOnce(() -> drivetrain.seedFieldCentric()));
        NamedCommands.registerCommand("zerohood", Commands.runOnce(() -> s_Hood.getInstance().setDegrees(0)));
        NamedCommands.registerCommand("stopspindexer",
                Commands.runOnce(() -> H_Spindex.setDesiredState(Spindex.SpindexStates.IDLE)));
        NamedCommands.registerCommand("track turret", Commands.runOnce(() -> {
            Pose2d goalPose = new Pose2d(4.620419, 4.034631, new Rotation2d());

            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    H_Turret.setGoal(() -> FlippingUtil.flipFieldPose(goalPose));
                    return;
                }
            }

            H_Turret.setGoal(() -> goalPose);

        }));

        NamedCommands.registerCommand("track left", (Commands.runOnce(() -> {
            Pose2d aimPose;
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    aimPose = new Pose2d(0.5, 6, new Rotation2d());

                } else {
                    aimPose = new Pose2d(16, 1.9, new Rotation2d());

                }
            } else {
                aimPose = new Pose2d(0.5, 1.6, new Rotation2d());

            }

            H_Turret.setGoal(() -> aimPose);

        })));

        NamedCommands.registerCommand("track right", (Commands.runOnce(() -> {
            Pose2d aimPose;
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    aimPose = new Pose2d(0.5, 1.6, new Rotation2d());

                } else {
                    aimPose = new Pose2d(16, 6, new Rotation2d());

                }
            } else {
                aimPose = new Pose2d(0.5, 1.6, new Rotation2d());

            }

            H_Turret.setGoal(() -> aimPose);
        })));

    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        // return drivetrain.runOnce(() ->
        // drivetrain.seedFieldCentric(Rotation2d.kZero));
        return currentAuto;

    }
}