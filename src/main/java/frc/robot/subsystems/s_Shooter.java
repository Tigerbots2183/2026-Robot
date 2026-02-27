// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;




public class s_Shooter extends SubsystemBase implements CheckableSubsystem {
  /** Creates a new s_Shooter. */
  public static s_Shooter m_Instance;

    private SparkFlex indexFlex = new SparkFlex(50, MotorType.kBrushless);

  private TalonFX leftTalonFlywheel = new TalonFX(7, "turret");
  private TalonFX rightTalonFlywheel = new TalonFX(6, "turret");

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withFollowers(Pair.of(rightTalonFlywheel, true))
      // Feedback Constants (PID Constants)
      .withClosedLoopController(.8, 0, 0)
      .withSimClosedLoopController(.8, 0, 0)
      // Feedforward Constants
      
      .withFeedforward(new SimpleMotorFeedforward(0, 0.1304, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0.1304, 0))
      // Telemetry name and verbosity level
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromTeeth(38, 40)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  // Create our SmartMotorController from our Talon.
  private SmartMotorController motorController = new TalonFXWrapper(leftTalonFlywheel, DCMotor.getKrakenX60Foc(2), smcConfig);

  private final FlyWheelConfig shooterConfig = new FlyWheelConfig(motorController)
      // Diameter of the flywheel.
      .withDiameter(Inches.of(4))
      // Mass of the flywheel.
      .withMass(Pounds.of(4.3983595))
      // Maximum speed of the shooter.
      .withUpperSoftLimit(RPM.of(5000))
      // Telemetry name and verbosity for the arm.
      .withTelemetry("Flywheel", TelemetryVerbosity.HIGH);

  // Shooter Mechanism
  private FlyWheel shooter = new FlyWheel(shooterConfig);

  private double rpm = 0;
  private Command setShooter = shooter.run(()-> RPM.of(rpm)).ignoringDisable(true);
  private Command stopShooter = shooter.setVoltage(Volts.of(0)).ignoringDisable(true);


  public s_Shooter() {
    initialized = true;
  }



  public static s_Shooter getInstance() {
    if (m_Instance == null) {
      m_Instance = new s_Shooter();
    }
    return m_Instance;
  }

  public boolean initialized = false;

  public boolean checkSubsystem() {
    return getInitialized();
  }

  public void setIndexVolts(double volts) {
    indexFlex.setVoltage(volts);
  }

  public void setIndexSpeed(double dutyCycle) {
    indexFlex.set(dutyCycle);
  }

  public void setShooterCommand(){
    CommandScheduler.getInstance().schedule(setShooter);

  }

  public void setStopCommand(){
    CommandScheduler.getInstance().schedule(stopShooter);

  }

  public void setShooterVolts(double volts){
    shooter.setVoltage(Volts.of(volts));
  }

  public void setRPM(double rpm) {
    this.rpm = rpm;
  }

  public double getVelocity(){
    return shooter.getSpeed().in(RPM);
  }

  public boolean getInitialized() {
    return initialized;
  }

  public void stop() {

  }

  @Override
  public void periodic() {
    shooter.updateTelemetry();
    // This method will be called once per scheduler run
  }

  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    shooter.simIterate();
  }
}
