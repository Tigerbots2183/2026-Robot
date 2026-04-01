// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.concurrent.ThreadLocalRandom;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.handlers.Turret;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Touchboard.JukeboxUtil;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    

    private final RobotContainer m_robotContainer;
    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        addPeriodic(()->m_robotContainer.H_Spindex.update(), 0.01);

    }


    @Override
    public void robotPeriodic() {

        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
    }

    private String[] songs = { "AnotherMedium.chrp", "Athletic.chrp", "FallenDown.chrp", "GustyGardenGalaxy.chrp",
            "Moon.chrp", "Otherside.chrp", "Ruins.chrp", "Sg.chrp", "spj.chrp", "SuperMarioLand.chrp",
            "SuperMarioWorldEnding.chrp", "WorkingForNook.chrp" };

    @Override
    public void disabledInit() {

        // mJukebox.mOrchestra.play();
    }

  
    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    // TurretIO test = TurretIO.getInstance();

    @Override
    public void teleopInit() {
        
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        // Supplier<Pose2d> goalPosition = () -> new Pose2d(4.620419, 4.034631, new Rotation2d());

        // if (DriverStation.getAlliance().isPresent()) {
        //     if (DriverStation.getAlliance().get() == Alliance.Red) {
        //         goalPosition = () -> FlippingUtil.flipFieldPose(new Pose2d(4.620419, 4.034631, new Rotation2d()));

        //     }
        // }

        // Turret.getInstance().setGoal(goalPosition);
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {
    }
        JukeboxUtil jukebox = new JukeboxUtil();

    @Override
    public void testInit() {
        SignalLogger.start();
        CommandScheduler.getInstance().cancelAll();
        jukebox.mOrchestra.stop();
        CommandSwerveDrivetrain drivetrain = TunerConstants.getInstance();

        jukebox.addTalon(drivetrain.getModule(0).getDriveMotor());
        jukebox.addTalon(drivetrain.getModule(1).getDriveMotor());
        jukebox.addTalon(drivetrain.getModule(2).getDriveMotor());
        jukebox.addTalon(drivetrain.getModule(3).getDriveMotor());

        jukebox.addTalon(drivetrain.getModule(0).getSteerMotor());
        jukebox.addTalon(drivetrain.getModule(1).getSteerMotor());
        jukebox.addTalon(drivetrain.getModule(2).getSteerMotor());
        jukebox.addTalon(drivetrain.getModule(3).getSteerMotor());

        jukebox.mOrchestra.loadMusic(songs[ThreadLocalRandom.current().nextInt(songs.length)] );
        jukebox.mOrchestra.play();

        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(()-> jukebox.mOrchestra.stop()));
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(()-> jukebox.mOrchestra.stop()));

        // Turret.getInstance().setDesiredState(Turret.TurretStates.SYSID);
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
