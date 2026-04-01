// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.handlers.Spindex;

public class s_Index extends SubsystemBase implements CheckableSubsystem {
  /** Creates a new s_Shooter. */
  public static s_Index m_Instance;

  private TalonFX indexTalon = new TalonFX(50);
  final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

  public s_Index() {
    initialized = true;

    // in init function
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output

    // set Motion Magic Velocity settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    indexTalon.getConfigurator().apply(talonFXConfigs);
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

  boolean jamming = false;
  Spindex serializer = Spindex.getInstance();

  public void setIndexRpmAndUnjam(double rpm) {

    if (indexTalon.getVelocity().getValue().in(RPM) < 100 && jamming == false) {
      indexTalon.setControl(m_request.withVelocity(RPM.of(rpm)));
      serializer.setDesiredState(Spindex.SpindexStates.REVERSE);
      jamming = true;

    } else if (jamming = true) {
      indexTalon.setControl(m_request.withVelocity(RPM.of(rpm)));
      serializer.setDesiredState(Spindex.SpindexStates.FEEDING);
      jamming = false;

    }
  }

  public void setIndexRpm(double rpm) {
    indexTalon.setControl(m_request.withVelocity(RPM.of(rpm)));

    if (rpm == 0) {
      serializer.setDesiredState(Spindex.SpindexStates.IDLE);
    }
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

  public double getVelocity() {
    return indexTalon.getVelocity().getValue().in(RPM);
  }

  public boolean getInitialized() {
    return initialized;
  }

  public void stop() {

  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
