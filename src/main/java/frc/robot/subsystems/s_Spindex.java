// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.deser.impl.BeanPropertyMap;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.handlers.Spindex;

public class s_Spindex extends SubsystemBase implements CheckableSubsystem {
  /** Creates a new s_Spindex. */
  public static s_Spindex m_Instance;

  public s_Spindex() {
    initialized = true;
  }

  private SparkFlex SpindexFlexLeft = new SparkFlex(40, MotorType.kBrushless); // black wheel
  private DigitalInput beamBreakLeft = new DigitalInput(9);

  private SparkFlex SpindexFlexRight = new SparkFlex(41, MotorType.kBrushless); // blue wheel
  private DigitalInput beamBreakRight = new DigitalInput(8);

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

  final double primaryVoltage = 3;
  final double secondaryVoltage = 1.5;
  double rounded;
  public void setFromBeamBreaks() {

    if (!beamBreakLeft.get() && !beamBreakRight.get()) {
      rounded = Math.round(Timer.getTimestamp() * 3) / 3.0;
      if ((rounded % 1) == 0) {
        SpindexFlexLeft.setVoltage(primaryVoltage);
        SpindexFlexRight.setVoltage(secondaryVoltage);
      } else {
        SpindexFlexLeft.setVoltage(-secondaryVoltage);
        SpindexFlexRight.setVoltage(-primaryVoltage);
      }
    } else if (!beamBreakLeft.get() && beamBreakRight.get()) {
      SpindexFlexLeft.setVoltage(-primaryVoltage);
      SpindexFlexRight.setVoltage(-secondaryVoltage);

    } else if (!beamBreakRight.get() && beamBreakLeft.get()) {

      SpindexFlexLeft.setVoltage(secondaryVoltage);
      SpindexFlexRight.setVoltage(primaryVoltage);

    } else {
            SpindexFlexLeft.setVoltage(-primaryVoltage);
      SpindexFlexRight.setVoltage(primaryVoltage);
      // rounded = Math.round(Timer.getTimestamp() * 2) / 2.0;
      // if ((rounded % 1) == 0) {
      //   SpindexFlexLeft.setVoltage(primaryVoltage);
      //   SpindexFlexRight.setVoltage(secondaryVoltage);
      // } else {
      //   SpindexFlexLeft.setVoltage(-secondaryVoltage);
      //   SpindexFlexRight.setVoltage(-primaryVoltage);
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
