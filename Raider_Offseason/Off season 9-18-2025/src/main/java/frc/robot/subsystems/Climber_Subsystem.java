// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

public class Climber_Subsystem extends SubsystemBase {
  private DoubleSubscriber NTSpeedClimber;
  private DoublePublisher ClimberSpeed;

  private static class Constants {
    public static int CLIMBER_FX_CANID = 16;
  }

  private static TalonFX climberFx = new TalonFX(Constants.CLIMBER_FX_CANID, "Upper Deck");

  final VelocityVoltage climber_request = new VelocityVoltage(0).withSlot(0);

  /** Creates a new Shooter_Subsystem. */
  public Climber_Subsystem() {
    NetworkTable NT = NetworkTableInstance.getDefault().getTable("climber");
    NTSpeedClimber = NT.getDoubleTopic("ClimberSpeed").subscribe(0.0);

    ClimberSpeed = NT.getDoubleTopic("ClimberSpeed").publish();
    ClimberSpeed.set(0.0);

    // in init function, set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    climberFx.getConfigurator().apply(slot0Configs);
  }

  private void setMotorSpeed(double speed) {
    climberFx.setControl(climber_request.withVelocity(speed));
  }

  private double getClimberSpeed() {
    return climberFx.getVelocity().getValueAsDouble();
  }

  private void NTUpdate() {
    ClimberSpeed.set(getClimberSpeed());

    SignalLogger.writeDouble("ClimberSpeed", getClimberSpeed());
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

  public Command climberDown() {
    return setVelocityCommand(-30);
  }

  public Command climberUp() {
    return setVelocityCommand(30);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
