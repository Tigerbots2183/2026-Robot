// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class s_Shooter extends SubsystemBase implements CheckableSubsystem {
  /** Creates a new s_Shooter. */
  public static s_Shooter m_Instance;

  public s_Shooter() {
    initialized = true;
  }

  private SparkFlex indexFlex = new SparkFlex(50, MotorType.kBrushless);

  private TalonFX leftTalonFlywheel = new TalonFX(7, "turret");
  private TalonFX rightTalonFlywheel = new TalonFX(6, "turret");


  public static s_Shooter getInstance(){
    if (m_Instance == null) {
      m_Instance = new s_Shooter();
    }
    return m_Instance;
  }


  public boolean initialized = false; 

  public boolean checkSubsystem(){
    return getInitialized();
  }

  public void setIndexVolts(double volts){
    indexFlex.setVoltage(volts);
  }

    public void setIndexSpeed(double dutyCycle){
    indexFlex.set(dutyCycle);
  }


  public void setShooterVolts(double volts){
    leftTalonFlywheel.set(volts);
    rightTalonFlywheel.set(-volts);

    //.53 /65%
  }

  public boolean getInitialized(){
    return initialized;
  }

  public void stop(){

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
