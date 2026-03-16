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
import frc.robot.Telemetry;
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

public class s_Index extends SubsystemBase implements CheckableSubsystem {
  /** Creates a new s_Shooter. */
  public static s_Index m_Instance;

  private TalonFX indexTalon = new TalonFX(50);

  private double rpmIndex = 0;

  private SmartMotorControllerConfig indexSMCConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(.2, 0, 0)
      .withSimClosedLoopController(.1, 0, 0)
      // Feedforward Constants
      .withGearing(new MechanismGearing(1,1))

      .withFeedforward(new SimpleMotorFeedforward(0, 0.134, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0.134, 0))
      // Telemetry name and verbosity level
      // .withTelemetry("ShooterMotor", TelemetryVerbosity.LOW)
      // Motor properties to prevent over currenting.
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("IndexMotor", TelemetryVerbosity.HIGH)
      .withSupplyCurrentLimit(Amps.of(70));

  // Create our SmartMotorController from our Talon.
  private SmartMotorController indexController = new TalonFXWrapper(indexTalon, DCMotor.getKrakenX60(1),
      indexSMCConfig);

  private final FlyWheelConfig indexConfig = new FlyWheelConfig(indexController)
      // Diameter of the flywheel.
      .withDiameter(Inches.of(2))
      // Mass of the flywheel.
      .withMass(Pounds.of(0.8))
      // Maximum speed of the shooter.
      .withUpperSoftLimit(RPM.of(5000))
      .withTelemetry("Index", TelemetryVerbosity.HIGH);

  // Telemetry name and verbosity for the arm.
  // .withTelemetry("Flywheel", TelemetryVerbosity.LOW);

  private FlyWheel index = new FlyWheel(indexConfig);

  private Command setIndex = index.run(() -> RPM.of(rpmIndex)).ignoringDisable(true);

  public s_Index() {
    initialized = true;
  }

  public static s_Index getInstance() {
    if (m_Instance == null) {
      m_Instance = new s_Index();
    }
    return m_Instance;
  }

  public boolean initialized = false;

  public boolean checkSubsystem() {
    return getInitialized();
  }

  public void setIndexRpm(DoubleSupplier rpm) {
    this.rpmIndex = rpm.getAsDouble();
  }

  public void setIndexRpm(double rpm) {
    this.rpmIndex = rpm;
  }

  // public void setIndexVolts(double volts) {
  // indexTalon.setVoltage(-volts);

  // }
  // public void setIndexVolts(DoubleSupplier volts) {
  // indexTalon.setVoltage(-volts.getAsDouble());
  // }
  // public void setIndexSpeed(double dutyCycle) {
  // indexTalon.set(-dutyCycle);
  // }
  // public void setIndexSpeed(DoubleSupplier dutyCycle) {
  // indexTalon.set(-dutyCycle.getAsDouble());
  // }
  public void setIndexCommand() {
    CommandScheduler.getInstance().schedule(setIndex);

  }

  public double getVelocity(){
    return index.getSpeed().in(RPM);
  }

  


  public boolean getInitialized() {
    return initialized;
  }

  public void stop() {

  }

  @Override
  public void periodic() {
    // shooter.updateTelemetry();
    index.updateTelemetry();

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    index.simIterate();
  }
}
