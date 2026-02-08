// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.ThreadLocalRandom;
import java.util.function.Supplier;

import com.ctre.phoenix6.HootAutoReplay;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.IO.TurretIO;
import frc.robot.handlers.Turret;
import frc.robot.subsystems.Touchboard.JukeboxUtil;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    private static JukeboxUtil mJukebox = JukeboxUtil.getInstance();
    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
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

        mJukebox.mOrchestra.loadMusic(songs[ThreadLocalRandom.current().nextInt(songs.length)]);
        // mJukebox.mOrchestra.loadMusic("pyramids.chrp");
        // //songs[ThreadLocalRandom.current().nextInt(songs.length)] );

        mJukebox.mOrchestra.play();
    }

    @Override
    public void disabledPeriodic() {
        if (!mJukebox.mOrchestra.isPlaying()) {
            mJukebox.mOrchestra.loadMusic(songs[ThreadLocalRandom.current().nextInt(songs.length)]);

            // mJukebox.mOrchestra.loadMusic("pyramids.chrp");
            // //songs[ThreadLocalRandom.current().nextInt(songs.length)] );
            mJukebox.mOrchestra.play();
        }
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        mJukebox.mOrchestra.stop();

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

    TurretIO test = TurretIO.getInstance();

    @Override
    public void teleopInit() {
        mJukebox.mOrchestra.stop();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        Supplier<Pose2d> goalPosition = ()-> new Pose2d(4.620419, 4.034631, new Rotation2d());

        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                goalPosition = ()-> FlippingUtil.flipFieldPose(new Pose2d(4.620419, 4.034631, new Rotation2d()));

            }
        }

        Turret.getInstance().setGoal( goalPosition);
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
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
