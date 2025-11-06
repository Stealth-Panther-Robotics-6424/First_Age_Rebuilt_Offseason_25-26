// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.Flow.Publisher;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter_Subsystem extends SubsystemBase {
  private DoubleSubscriber NTSpeedTop;
  private DoubleSubscriber NTSpeedBottom;
  private DoublePublisher TopSpeed;
  private DoublePublisher BottomSpeed;

  private static class Constants {
    public static int TOP_FX_CANID = 15;
    public static int BOTTOM_FX_CANID = 10;
  }

  private static TalonFX topFx = new TalonFX(Constants.TOP_FX_CANID, "Upper Deck");
  private static TalonFX bottomFx = new TalonFX(Constants.BOTTOM_FX_CANID, "Upper Deck");

  /** Creates a new Shooter_Subsystem. */
  public Shooter_Subsystem() {
    NetworkTable NT = NetworkTableInstance.getDefault().getTable("shooter");
    NTSpeedTop = NT.getDoubleTopic("TopSpeed").subscribe(0.0);
    NTSpeedBottom = NT.getDoubleTopic("BottomSpeed").subscribe(0.0);

    TopSpeed = NT.getDoubleTopic("TopSpeed").publish();
    BottomSpeed = NT.getDoubleTopic("BottomSpeed").publish();
    TopSpeed.set(0.0);
    BottomSpeed.set(0.0);
  }

  private void setMotorPower(double power) {
    topFx.set(power);
    bottomFx.set(power);

  }

  private double getTopSpeed() {
    return topFx.getVelocity().getValueAsDouble();
  }

  private double getBottomSpeed() {
    return bottomFx.getVelocity().getValueAsDouble();
  }

  private void NTUpdate() {
    TopSpeed.set(getTopSpeed());
    BottomSpeed.set(getBottomSpeed());
  }

  public Command runMotorCommand(DoubleSupplier powerAxis) {
    return new FunctionalCommand(
        () -> {

        },

        () -> {
          setMotorPower(powerAxis.getAsDouble());

        },

        interrupted -> {
          setMotorPower(0);

        },

        () -> false,
        this);
  }

  @Override
  public void periodic() {
    NTUpdate();

  }
}
