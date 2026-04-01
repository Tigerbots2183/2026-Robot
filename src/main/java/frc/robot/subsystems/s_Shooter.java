// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.RPM;

public class s_Shooter extends SubsystemBase implements CheckableSubsystem {
  /** Creates a new s_Shooter. */
  public static s_Shooter m_Instance;

  private TalonFX leftTalonFlywheel = new TalonFX(7, "turret");
  private TalonFX rightTalonFlywheel = new TalonFX(6, "turret");

  final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

  public double offset = 0; 

  public s_Shooter() {
    initialized = true;

    var talonFXConfigs = new TalonFXConfiguration();
    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.1504; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = .1; // A position error of 2.5 rotations results in 12 V output

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 50;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    var supplyLimitConfig = new TalonFXConfiguration();

    supplyLimitConfig.CurrentLimits.SupplyCurrentLimit = 50;
    supplyLimitConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    leftTalonFlywheel.getConfigurator().apply(talonFXConfigs);

    rightTalonFlywheel.setControl(new Follower(7, MotorAlignmentValue.Opposed));
    rightTalonFlywheel.getConfigurator().apply(supplyLimitConfig);
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

  public void setShooterVolts(double volts) {
    // shooter.setVoltage(Volts.of(volts));
    leftTalonFlywheel.setVoltage(volts);
  }

  public void setRPM(double rpm) {
    leftTalonFlywheel.setControl(m_request.withVelocity(RPM.of(rpm + offset)));
  }

  public double getVelocity() {
    return leftTalonFlywheel.getVelocity().getValue().in(RPM) + offset;
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
