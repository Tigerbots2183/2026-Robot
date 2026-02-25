// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.xml.namespace.QName;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.s_Intake;
import frc.robot.subsystems.Touchboard.JukeboxUtil;

public class RobotContainer {

    private Intake H_Intake = Intake.getInstance();
    private Spindex H_Spindex = Spindex.getInstance();
    private Drivetrain H_Drivetrain = Drivetrain.getInstance();
    private Shooter H_Shooter = Shooter.getInstance();
    // private QuestNavSubsystem Q_Nav = QuestNavSubsystem.getInstance();

    // private Turret H_Turret = Turret.getInstance();
    private Hood H_Hood = Hood.getInstance();

    

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.getInstance();

    public RobotContainer() {
         
        configureBindings();
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(()->{
            if(RobotBase.isSimulation()){
                drivetrain.runOnce(drivetrain::seedFieldCentric).ignoringDisable(true);
            }
        }));

        
        // RobotModeTriggers.autonomous().onTrue(Commands.runOnce(()-> Q_Nav.setInitialPose()));
    }

    private void configureBindings() {

        joystick.rightBumper().onTrue(Commands.runOnce(() -> H_Intake.setDesiredState(Intake.IntakeStates.INTAKING)));
        joystick.rightBumper().onFalse(Commands.runOnce(() -> H_Intake.setDesiredState(Intake.IntakeStates.OUT)));


        joystick.y().onTrue(Commands.runOnce(() -> H_Spindex.setDesiredState(Spindex.SpindexStates.FEEDING)));

        joystick.y().onFalse(Commands.runOnce(() -> H_Spindex.setDesiredState(Spindex.SpindexStates.IDLE)));

        joystick.y().onTrue(Commands.runOnce(() -> H_Shooter.setDesiredState(Shooter.ShooterStates.SHOOTING)));
        joystick.y().onTrue(Commands.runOnce(() -> H_Intake.setDesiredState(Intake.IntakeStates.RAISING)));
        
        joystick.y().onFalse(Commands.runOnce(() -> H_Shooter.setDesiredState(Shooter.ShooterStates.IDLE)));
        joystick.y().onFalse(Commands.runOnce(() -> H_Intake.setDesiredState(Intake.IntakeStates.OUT)));


        
        // joystick.pov(180).onTrue(AutoBuilder.pathfindToPose(new Pose2d(8,4, new Rotation2d()), CommandSwerveDrivetrain.pConstraints, 4.0));
        // joystick.pov(0).onTrue(Commands.runOnce(() -> H_Intake.setDesiredState(Intake.IntakeStates.RAISING)));
        // joystick.pov(0).onFalse(Commands.runOnce(() -> H_Intake.setDesiredState(Intake.IntakeStates.OUT)));

        // joystick.pov(90).onTrue(Commands.runOnce(() -> H_Intake.setDesiredState(Intake.IntakeStates.HUMAN)));
        // joystick.pov(90).onFalse(Commands.runOnce(() -> H_Intake.setDesiredState(Intake.IntakeStates.IDLE)));

        // joystick.pov(180).onTrue(Commands.runOnce(() -> H_Intake.setDesiredState(Intake.IntakeStates.IDLE)));
    
        joystick.pov(180).onTrue(Commands.runOnce(()-> H_Hood.decreaseDeg()));
        joystick.pov(0).onTrue(Commands.runOnce(()-> H_Hood.increaseDeg()));
        

        joystick.b().onFalse(Commands.runOnce(() -> H_Shooter.setDesiredState(Shooter.ShooterStates.IDLE)));

        joystick.a().onFalse(Commands.runOnce(() -> H_Spindex.setDesiredState(Spindex.SpindexStates.IDLE)));
        joystick.a().onFalse(Commands.runOnce(() -> H_Shooter.setDesiredState(Shooter.ShooterStates.IDLE)));
        joystick.a().onTrue(Commands.runOnce(() -> H_Intake.setDesiredState(Intake.IntakeStates.IDLE)));



        joystick.a().onTrue(Commands.runOnce(() -> H_Spindex.setDesiredState(Spindex.SpindexStates.REVERSE)));
        joystick.a().onTrue(Commands.runOnce(() -> H_Intake.setDesiredState(Intake.IntakeStates.REVERSE)));

        drivetrain.runOnce(drivetrain::seedFieldCentric).ignoringDisable(true);

        JukeboxUtil jukebox = JukeboxUtil.getInstance();
        jukebox.addTalon(drivetrain.getModule(0).getDriveMotor());
        jukebox.addTalon(drivetrain.getModule(1).getDriveMotor());
        jukebox.addTalon(drivetrain.getModule(2).getDriveMotor());
        jukebox.addTalon(drivetrain.getModule(3).getDriveMotor());

        jukebox.addTalon(drivetrain.getModule(0).getSteerMotor());
        jukebox.addTalon(drivetrain.getModule(1).getSteerMotor());
        jukebox.addTalon(drivetrain.getModule(2).getSteerMotor());
        jukebox.addTalon(drivetrain.getModule(3).getSteerMotor());

        jukebox.addTalon(s_Intake.getInstance().getLeftPivotTalonFX());
        jukebox.addTalon(s_Intake.getInstance().getRightPivotTalonFx());

        // 
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        return drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero));

    }
}
