// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class s_Example extends SubsystemBase implements CheckableSubsystem {
  /** Creates a new s_Example. */
  public static s_Example m_Instance;

  public s_Example() {
    initialized = true;
  }



  public static s_Example getInstance(){
    if (m_Instance == null) {
      m_Instance = new s_Example();
    }
    return m_Instance;
  }


  public boolean initialized = false; 

  public boolean checkSubsystem(){
    return getInitialized();
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
