// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.Flow.Publisher;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// pid code
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

  final VelocityVoltage top_request = new VelocityVoltage(0).withSlot(0);
  final VelocityVoltage bottom_request = new VelocityVoltage(0).withSlot(0);

  /** Creates a new Shooter_Subsystem. */
  public Shooter_Subsystem() {
    NetworkTable NT = NetworkTableInstance.getDefault().getTable("shooter");
    NTSpeedTop = NT.getDoubleTopic("TopSpeed").subscribe(0.0);
    NTSpeedBottom = NT.getDoubleTopic("BottomSpeed").subscribe(0.0);

    TopSpeed = NT.getDoubleTopic("TopSpeed").publish();
    BottomSpeed = NT.getDoubleTopic("BottomSpeed").publish();
    TopSpeed.set(0.0);
    BottomSpeed.set(0.0);

    // in init function, set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    topFx.getConfigurator().apply(slot0Configs);
    bottomFx.getConfigurator().apply(slot0Configs);
  }

  private void setMotorPower(double power) {
    topFx.set(power);
    bottomFx.set(power);

  }

  private void setMotorSpeed(double speed) {
    topFx.setControl(top_request.withVelocity(speed));
    bottomFx.setControl(bottom_request.withVelocity(speed));
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

    SignalLogger.writeDouble("TopSpeed", getTopSpeed());
    SignalLogger.writeDouble("BottomSpeed", getBottomSpeed());
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

  public Command setVelocityCommand(double speed) {
    return new FunctionalCommand(
        () -> {
          setMotorSpeed(speed);

        },

        () -> {

        },

        interrupted -> {
          setMotorSpeed(0);

        },

        () -> false,
        this);
  }

  public Command highSpeed() {
    return setVelocityCommand(-50);
  }

  public Command slowReverse() {
    return setVelocityCommand(10);
  }

  @Override
  public void periodic() {
    NTUpdate();

  }
}
