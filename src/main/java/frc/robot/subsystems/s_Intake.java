// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;
import static yams.units.YUnits.PoundSquareInches;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;


public class s_Intake extends SubsystemBase implements CheckableSubsystem {
  /** Creates a new s_Intake. */
  public static s_Intake m_Instance;



  
  public s_Intake() {
    initialized = true;
  }
  // Vendor motor controller object
  private TalonFX lPivotTalonFX = new TalonFX(30);
  private TalonFX rPivotTalonFX = new TalonFX(31);

   private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  .withFollowers(Pair.of(rPivotTalonFX, false))
  // Feedback Constants (PID Constants)
  .withClosedLoopController(100, 0, 0, DegreesPerSecond.of(280), DegreesPerSecondPerSecond.of(365))
  .withSimClosedLoopController(100, 0, 0, DegreesPerSecond.of(280), DegreesPerSecondPerSecond.of(365))
  // Feedforward Constants
  .withFeedforward(new ArmFeedforward(0, 0, 0))
  .withSimFeedforward(new ArmFeedforward(0, 0, 0))
  // Telemetry name and verbosity level
  .withTelemetry("Intake L R Motor", TelemetryVerbosity.LOW)
  // Gearing from the motor rotor to final shaft.
  // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.BRAKE)
  .withStatorCurrentLimit(Amps.of(70))
  .withClosedLoopRampRate(Seconds.of(0.05))
  .withOpenLoopRampRate(Seconds.of(0.05));




  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController talonSmartMotorController = new TalonFXWrapper(lPivotTalonFX, DCMotor.getKrakenX60Foc(2), smcConfig);

  private ArmConfig armCfg = new ArmConfig(talonSmartMotorController)
  // Soft limit is applied to the SmartMotorControllers PID
  .withSoftLimits(Degrees.of(-125), Degrees.of(135))
  // // Hard limit is applied to the simulation.
  .withHardLimit(Degrees.of(-125), Degrees.of(135))
  // Starting position is where your arm starts
  .withStartingPosition(Degrees.of(0))
  // Length and mass of your arm for sim.
  .withLength(Inches.of(13.5))
  .withMOI(MomentOfInertia.ofRelativeUnits(1265.329267,  PoundSquareInches))
  // Telemetry name and verbosity for the arm.
  .withTelemetry("Intake", TelemetryVerbosity.LOW);

  // Arm Mechanism
  private Arm intake = new Arm(armCfg);

  private SparkFlex intakeRoller = new SparkFlex(32, MotorType.kBrushless);

  private double currentDeg = 0.0;
  private Command intakeSetter = intake.setAngle(() -> Degrees.of(currentDeg)).ignoringDisable(true);

  public static s_Intake getInstance(){
    if (m_Instance == null) {
      m_Instance = new s_Intake();
    }
    return m_Instance;
  }


  public void setDegreeCommand(){
    CommandScheduler.getInstance().schedule(intakeSetter);
  }

  public void setDegrees(double deg){
    // intake.setAngle(Degrees.of(deg)).schedule();
    currentDeg = deg;
  }

  // public void overrideDeg(double deg){
  // }

  public void setSpeed(double dutyCycle){
    intakeRoller.set(dutyCycle);
  }

  public TalonFX getLeftPivotTalonFX(){
    return lPivotTalonFX;
  }

  public TalonFX getRightPivotTalonFx(){
    return rPivotTalonFX;
  }

  public boolean initialized = false; 

  public boolean checkSubsystem(){
    return getInitialized();
  }

  public boolean getInitialized(){
    return initialized;
  }

  public void stop(){
    intakeRoller.stopMotor();
    lPivotTalonFX.stopMotor();
    rPivotTalonFX.stopMotor();
  }

  @Override
  public void periodic() {
    // intake.updateTelemetry();
    // This method will be called once per scheduler run
  }

  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    intake.simIterate();
  }
}
