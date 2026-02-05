// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class s_Spindex extends SubsystemBase implements CheckableSubsystem {
  /** Creates a new s_Spindex. */
  public static s_Spindex m_Instance;

  public s_Spindex() {
    initialized = true;
  }

  private SparkFlex SpindexFlex = new SparkFlex(40, MotorType.kBrushless);

  public static s_Spindex getInstance(){
    if (m_Instance == null) {
      m_Instance = new s_Spindex();
    }
    return m_Instance;
  }

  public void setVoltage(double volts){
    SpindexFlex.setVoltage(volts);
  }

  public boolean initialized = false; 

  public boolean checkSubsystem(){
    return getInitialized();
  }

  public boolean getInitialized(){
    return initialized;
  }

  public void stop(){
    SpindexFlex.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
