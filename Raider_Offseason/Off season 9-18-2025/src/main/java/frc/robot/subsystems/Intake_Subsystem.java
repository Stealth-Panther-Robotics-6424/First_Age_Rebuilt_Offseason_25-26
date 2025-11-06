// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake_Subsystem extends SubsystemBase {
  private DoubleSubscriber NTHSpeed;
  private DoublePublisher HSpeed;

  private static class Constants {
    public static int HINTAKE_FX_CANID = 15;
  }

  private static TalonFX hIntakeFx = new TalonFX(Constants.HINTAKE_FX_CANID, "Upper Deck");

  /** Creates a new Intake_Subsystem. */
  public Intake_Subsystem() {
    /*
     * NetworkTable NT = NetworkTableInstance.getDefault().getTable("Intake");
     * NTHSpeed = NT.getDoubleTopic("HSpeed").subscribe(0.0);
     * 
     * HSpeed = NT.getDoubleTopic("HSpeed").publish();
     * HSpeed.set(0.0);
     * }
     * 
     * private void setMotorPower(double power) {
     * hIntakeFx.set(power);
     * 
     * }
     * 
     * private double getHSpeed() {
     * return hIntakeFx.getVelocity().getValueAsDouble();
     * }
     * 
     * private void NTUpdate() {
     * HSpeed.set(getHSpeed());
     * }
     * 
     * public Command runMotorCommand(DoubleSupplier powerAxis) {
     * return new FunctionalCommand(
     * () -> {
     * 
     * },
     * 
     * () -> {
     * setMotorPower(powerAxis.getAsDouble());
     * 
     * },
     * 
     * interrupted -> {
     * setMotorPower(0);
     * 
     * },
     * 
     * () -> false,
     * this);
     */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
