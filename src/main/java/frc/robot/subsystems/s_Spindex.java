// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class s_Spindex extends SubsystemBase implements CheckableSubsystem {
  /** Creates a new s_Spindex. */
  public static s_Spindex m_Instance;

  private SparkFlex SpindexFlexLeft = new SparkFlex(40, MotorType.kBrushless); // black wheel
  private DigitalInput beamBreakLeft = new DigitalInput(9);

  private SparkFlex SpindexFlexRight = new SparkFlex(41, MotorType.kBrushless); // blue wheel
  private DigitalInput beamBreakRight = new DigitalInput(8);

  private SparkClosedLoopController m_ControllerLeft = SpindexFlexLeft.getClosedLoopController();
  private SparkClosedLoopController m_ControllerRight = SpindexFlexRight.getClosedLoopController();

  private SparkFlexConfig config = new SparkFlexConfig();
  private SparkFlexConfig config2 = new SparkFlexConfig();

  private RelativeEncoder leftEncoder = SpindexFlexLeft.getEncoder();
  private RelativeEncoder rightEncoder = SpindexFlexRight.getEncoder();


  public s_Spindex() {
    initialized = true;
    config.closedLoop.p(0.0001).i(0).d(0.0);
    config2.closedLoop.p(0.0001).i(0).d(0.0);
    config.closedLoop.feedForward.kV(0.0018);
    config2.closedLoop.feedForward.kV(0.0018);

    config.smartCurrentLimit(70);
    config2.smartCurrentLimit(70);


    SpindexFlexLeft.configure(config, com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
    SpindexFlexRight.configure(config2, com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);

  }

  public static s_Spindex getInstance() {
    if (m_Instance == null) {
      m_Instance = new s_Spindex();
    }
    return m_Instance;
  }

  public void setVoltage(double volts) {
    SpindexFlexLeft.setVoltage(volts);
    SpindexFlexRight.setVoltage(volts);

  }

  public void setDiffVoltage(double volts) {
    SpindexFlexLeft.setVoltage(-volts);
    SpindexFlexRight.setVoltage(volts);

  }

  // final double primaryVoltage = 6;
  // final double secondaryVoltage = 3;

  final double primarySetpoint = 4000;
  final double secondarySetpoint = 500;
  double rounded;

  // public double timeout = 100; 

  public void setFromBeamBreaks() {
    // timeout--;

    // if((Math.abs(leftEncoder.getVelocity()) < 20) || (Math.abs(rightEncoder.getVelocity()) < 20)&& (beamBreakLeft.get() || beamBreakRight.get()) && timeout <= 0){
    //   m_ControllerLeft.setSetpoint(primarySetpoint, ControlType.kVelocity);
    //   m_ControllerRight.setSetpoint(secondarySetpoint, ControlType.kVelocity);

    //   return;
    // } else if (!(Math.abs(leftEncoder.getVelocity()) < 20) || (Math.abs(rightEncoder.getVelocity()) < 20)) {
    //   timeout = 100;
    // }


    if (!beamBreakLeft.get() && !beamBreakRight.get()) {
      m_ControllerLeft.setSetpoint(primarySetpoint, ControlType.kVelocity);
      m_ControllerRight.setSetpoint(secondarySetpoint, ControlType.kVelocity);

      // SpindexFlexLeft.setVoltage(primaryVoltage);
      // SpindexFlexRight.setVoltage(secondaryVoltage);
      // rounded = Math.round(Timer.getTimestamp() * 3) / 3.0;
      // if ((rounded % 1) == 0) {
      // SpindexFlexLeft.setVoltage(primaryVoltage);
      // SpindexFlexRight.setVoltage(secondaryVoltage);
      // } else {
      // SpindexFlexLeft.setVoltage(-secondaryVoltage);
      // SpindexFlexRight.setVoltage(-primaryVoltage);
      // }
    } else if (!beamBreakLeft.get() && beamBreakRight.get()) {
      // SpindexFlexLeft.setVoltage(-primaryVoltage);
      // SpindexFlexRight.setVoltage(-secondaryVoltage);
      m_ControllerLeft.setSetpoint(-primarySetpoint, ControlType.kVelocity);
      m_ControllerRight.setSetpoint(-secondarySetpoint, ControlType.kVelocity);

    } else if (!beamBreakRight.get() && beamBreakLeft.get()) {
      m_ControllerLeft.setSetpoint(secondarySetpoint, ControlType.kVelocity);
      m_ControllerRight.setSetpoint(primarySetpoint, ControlType.kVelocity);
      // SpindexFlexLeft.setVoltage(secondaryVoltage);
      // SpindexFlexRight.setVoltage(primaryVoltage);

    } else {
      m_ControllerLeft.setSetpoint(-primarySetpoint, ControlType.kVelocity);
      m_ControllerRight.setSetpoint(primarySetpoint, ControlType.kVelocity);
      // SpindexFlexLeft.setVoltage(-primaryVoltage);
      // SpindexFlexRight.setVoltage(primaryVoltage);
      // rounded = Math.round(Timer.getTimestamp() * 2) / 2.0;
      // if ((rounded % 1) == 0) {
      // SpindexFlexLeft.setVoltage(primaryVoltage);
      // SpindexFlexRight.setVoltage(secondaryVoltage);
      // } else {
      // SpindexFlexLeft.setVoltage(-secondaryVoltage);
      // SpindexFlexRight.setVoltage(-primaryVoltage);
      // }
    }
  }

  public boolean initialized = false;

  public boolean checkSubsystem() {
    return getInitialized();
  }

  public boolean getInitialized() {
    return initialized;
  }

  public void stop() {
    SpindexFlexLeft.stopMotor();
    SpindexFlexRight.stopMotor();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
