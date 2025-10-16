// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter_Subsystem extends SubsystemBase {
  private static class Constants {
    public static int TOP_FX_CANID = 15;
    public static int BOTTOM_FX_CANID = 10;
  }

  private static TalonFX topFx = new TalonFX(Constants.TOP_FX_CANID);
  private static TalonFX bottomFx = new TalonFX(Constants.BOTTOM_FX_CANID);

  /** Creates a new Shooter_Subsystem. */
  public Shooter_Subsystem() {
  }

  private void setMotorPower(double power) {
    topFx.set(power);
    bottomFx.set(power);

  }

  public Command runMotorCommand(DoubleSupplier powerAxis) {
    return new FunctionalCommand(
        () -> {

        },

        () -> {
          setMotorPower(powerAxis.getAsDouble());

        },

        interrupted -> {

        },

        () -> false,
        this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
