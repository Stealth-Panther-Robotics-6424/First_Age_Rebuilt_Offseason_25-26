// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter_Subsystem extends SubsystemBase {

  private static class Constants {
   public static int TOP_FX_CANID = 15;    
  }

  private static TalonFX topFX = new TalonFX(Constants.TOP_FX_CANID);
  /** Creates a new Shooter_Subsystem. */
  public Shooter_Subsystem() {}

  private void setMotorPower (double power)
  {
    topFX 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
