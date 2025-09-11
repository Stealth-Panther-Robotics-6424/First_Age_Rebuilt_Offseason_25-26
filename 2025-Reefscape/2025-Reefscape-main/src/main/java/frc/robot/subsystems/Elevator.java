// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Import required classes
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /**
   * Creates a new Elevator subsystem. This is where the elevator motor control
   * and logic reside.
   */

  // Define the TalonFX motor controllers for both sides of the elevator (left and
  // right).
  private final TalonFX elevatorTalonPort = new TalonFX(Constants.ElevatorConstants.ElevatorPortMotor_ID);
  private final TalonFX elevatorTalonStrb = new TalonFX(Constants.ElevatorConstants.ElevatorStrbMotor_ID);

  // Flag to check if the elevator is allowed to lift based on a condition.
  private boolean canLift = false; // This variable sets whether the elevator can move up or down

  // Create a PID controller to control the elevator position, with initial PID
  // values (Proportional, Integral, Derivative).
  private final PIDController elevatorController = new PIDController(0.06, 0, 0); // The OG value was 0.0002348, but it
                                                                                  // has been adjusted

  // Setup Shuffleboard (WPILib's dashboard) for real-time monitoring of the
  // elevator state during the match.

  private ShuffleboardTab DS_ElevatorTab = Shuffleboard.getTab("Elevator");
  private GenericEntry DS_ElevatorPosition = DS_ElevatorTab.add("ElevatorValue", 0).getEntry(); // Entry for elevator
  // position

  private GenericEntry DS_ElevatorSpeed = DS_ElevatorTab.add("Elevator Speed",
      .2).getEntry(); // Entry for elevator
  // speed

  private GenericEntry DS_forwardLimit = DS_ElevatorTab.add("Forward Limit",
      true).getEntry(); // Entry for forward
  // limit

  private GenericEntry DS_reverseLimit = DS_ElevatorTab.add("Reverse Limit",
      true).getEntry(); // Entry for reverse
  // limit

  private GenericEntry DS_canLift = DS_ElevatorTab.add("CanLift",
      true).getEntry(); // Entry for canLift

  private GenericEntry DS_ElevatorSetpoint = DS_ElevatorTab.add("Setpoint",
      elevatorController.getSetpoint())
      .getEntry();

  // Default max elevator speed as defined by Shuffleboard.
  // double maxElevatorSpeed = this.DS_ElevatorSpeed.getDouble(0.2);

  public Elevator() {
    // Set both motor controllers to Coast mode to stop the motors from coasting
    // when no power is applied.
    elevatorTalonPort.setNeutralMode(NeutralModeValue.Coast);
    elevatorTalonStrb.setNeutralMode(NeutralModeValue.Coast);

    // Initialize motor configurations to apply current limits to avoid motor
    // overdraw.
    var ElevatorPortConfig = elevatorTalonPort.getConfigurator();
    var ElevatorStrbConfig = elevatorTalonStrb.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();

    // Apply current limits to motors, restricting the current draw to 60 Amps
    // (StatorCurrentLimit).
    limitConfigs.StatorCurrentLimit = 60;
    limitConfigs.StatorCurrentLimitEnable = true;

    ElevatorPortConfig.apply(limitConfigs); // Apply the configurations to the left motor
    ElevatorStrbConfig.apply(limitConfigs); // Apply the configurations to the right motor

    // Set initial PID controller setpoint to current elevator position.
    elevatorController.setSetpoint(getElevatorPosition());
    elevatorController.setTolerance(.67); // Set tolerance to 1 (tolerance defines when the PID controller considers
    // the
    // target reached)
    // elevatorTalonStrb.setPosition(0); // Possible initial position setting for
    // the second motor (commented out)
  }

  // Method to check if the elevator has reached its setpoint.
  public boolean elevatorAtSetpoint() {
    return this.elevatorController.atSetpoint(); // Returns true if the elevator is at its setpoint.
  }

  // Trigger that checks if the elevator is in the correct position for intake
  // (position <= 1).
  public Trigger elevatorIntake() {
    return new Trigger(() -> this.getElevatorPosition() <= 1.14);
  }

  // Method to get the current position of the elevator by reading the position
  // from the right motor.
  public double getElevatorPosition() {
    return (elevatorTalonStrb.getPosition().getValueAsDouble());

  }

  /*
   * Setters for the elevator motors.
   */

  // Helper method to set the power (speed) of a given motor.
  private void setMotorPower(TalonFX motor, double power) {
    motor.set(power); // Apply the desired power to the motor
  }

  // Method to set power to both elevator motors, considering limits.
  public void setElevatorMotor(double power) {
    double output = elevatorLimit(power);
    double kFValue = this.kfValueSetter();
    SmartDashboard.putNumber("LimitOutput", output); //
    // Apply limit on power to prevent the elevator from exceeding boundaries (e.g.,
    // going beyond the upper or lower limit).
    setMotorPower(elevatorTalonPort, -(output + kFValue)); // Apply power to the left motor with limits.
    setMotorPower(elevatorTalonStrb, output + kFValue); // Apply power to the right motor with limits.
    // we added kF value to here to make sure the elevator will hold when it is fed
    // a zero from its limiter

  }

  // Limit the motor power based on certain conditions such as the current
  // position of the elevator and whether it can lift.
  private double elevatorLimit(double power) {
    double output = 0; // Default output is 0 (no power).
    SmartDashboard.putNumber("Input", power);
    // If the elevator can't lift or if the elevator is at the top or bottom, set
    // the output power to a small value.
    if ((!canLift)
        || (elevatorTalonStrb.getPosition().getValueAsDouble() >= 65.71 && power > 0) // Positive Power makes the
        // robot go up negative makes the robot go down
        || (elevatorTalonStrb.getPosition().getValueAsDouble() <= 0.29 && power < 0)) {

      output = 0.0;
      // Output is zero but is given a kf value of .02 when it is applied to the motor
      // to allow the elevator to hold.
    } else {
      output = power;
      // Otherwise, apply the requested power to the motors.
    }
    return output; // Return the calculated output power.
  }

  // Methods for controlling the elevator using PID.
  public void setElevatorPID(double setPoint) {

    this.elevatorController.setSetpoint(setPoint); // Tells the PID controller what the setpoint is

  }

  public double kfValueSetter() {
    double kF = 0;
    if (this.getElevatorPosition() < 0.5) {
      kF = 0;
    } else {
      kF = .023;
    }

    return kF;
  }

  public double elevatorThrottle() {
    return ((1 - 0.5 * (this.getElevatorPosition() / 65.71))); // This method is used to slow the drivespeed down based
                                                               // on
    // the elevator position
    // any slower than this will make slowmode not be able to be held down on the
    // controller or the robot cannot move (OG value for division problem 115 new
    // value is 65.71)
  }

  public Trigger canFold() { // this method sets whether the wrist can fold back based on the elevator
                             // position this prevents folding back into the crossmembers
                             // Original values in order 26, 50.5, 59, 104
                             // new values in order 14.86, 28.86, 33.71, 59.43
    return new Trigger(() -> (!((this.getElevatorPosition() >= 14.86) && (this.getElevatorPosition() <= 28.86))
        || !((this.getElevatorPosition() >= 33.71) && (this.getElevatorPosition() <= 59.43))));

  }

  public Trigger reverseLimitHit() { // this method sets whether the wrist can fold back based on the elevator
    // position this prevents folding back into the crossmembers
    // Original values in order 26, 50.5, 59, 104
    // new values in order 14.86, 28.86, 33.71, 59.43
    return new Trigger(() -> (elevatorTalonStrb.getReverseLimit().getValueAsDouble() == 1));

  }

  public void zeroElevator() // Check if the built-in reverse limit
  {
    {
      elevatorTalonStrb.setPosition(0);
      elevatorTalonPort.setPosition(0); // Reset the value to zero
    }
  }

  // Method to calculate and apply the PID output to move the elevator towards the
  // setpoint.
  public void executeElevatorPID() {
    // Call the PID controller to calculate the required power and apply it to the
    // motors.
    double PIDValue = this.elevatorController.calculate(this.getElevatorPosition());

    setElevatorMotor(PIDValue);
  }

  // Command for manual control of the elevator during teleop, allowing the driver
  // to move the elevator with a joystick.
  public Command ManualElevator(DoubleSupplier elevatorJoystick, BooleanSupplier wristLimiter) {
    return new FunctionalCommand(
        () -> {
          this.canLift = wristLimiter.getAsBoolean(); // Update whether lifting is allowed based on wrist limiter.
        },

        () -> {
          // Use joystick input to control elevator power. Apply a scaling factor of 0.2
          // for smooth control.
          this.setElevatorMotor(elevatorJoystick.getAsDouble() * 0.4 + 0.1);// 0.4 + 0.1
          this.canLift = wristLimiter.getAsBoolean(); // Update lifting condition.
        },

        interrupted -> this.setElevatorPID(this.getElevatorPosition()), // If interrupted, reset the PID setpoint.

        () -> {
          return false; // No specific condition is required for the command to finish.
        },

        this);
  }

  // A command for starting the elevator with no movement but initializing
  // necessary states.
  public Command startCommand(BooleanSupplier wristLimiter) {
    return this.runOnce(() -> {
      this.canLift = wristLimiter.getAsBoolean(); // Update whether lifting is allowed.
      this.setElevatorPID(this.getElevatorPosition());
    } // Set the PID setpoint to current position.
    );
  }

  /*
   * public Command EndCommand(BooleanSupplier wristLimiter) {
   * return this.runOnce(() -> {
   * elevatorTalonPort.setNeutralMode(NeutralModeValue.Brake);
   * elevatorTalonStrb.setNeutralMode(NeutralModeValue.Brake);
   * });
   * }
   */

  public Command Zero() {
    return this.runOnce(() -> {
      this.zeroElevator();

    });
  }

  // Default PID elevator control command, continuously adjusting the elevator's
  // position based on the PID controller.
  public Command elevatorPIDCommandDefault(BooleanSupplier wristLimiter) {
    return new FunctionalCommand(
        () -> {
          this.canLift = wristLimiter.getAsBoolean(); // Update lifting condition.
        },

        () -> {
          this.canLift = wristLimiter.getAsBoolean();
          this.executeElevatorPID(); // Execute PID control to adjust the elevator's position.
        },

        interrupted -> {
          this.canLift = wristLimiter.getAsBoolean(); // Update lifting condition when interrupted.
        },

        () -> {
          return false; // No condition for command termination.
        },

        this);
  }

  // Command to move the elevator to a position based on a given setpoint. 1.09
  // rotations = 1 inch of height
  public Command MovetoPosition(BooleanSupplier wristLimiter, double position) {
    return new FunctionalCommand(
        () -> {
          this.canLift = wristLimiter.getAsBoolean(); // Update lifting condition.
          this.setElevatorPID(position); // Set PID setpoint to 0 (L1 position).
        },

        () -> {
          this.canLift = wristLimiter.getAsBoolean();
          this.executeElevatorPID(); // Execute PID control to move the elevator.
        },

        interrupted -> {
          this.setElevatorPID(this.getElevatorPosition());
          this.canLift = wristLimiter.getAsBoolean();
          this.setElevatorMotor(0);
        }, // Nothing new runs when interrupted

        () -> (this.elevatorAtSetpoint()), // Check if the elevator has reached L1.

        this);
  }

  // Command to move the elevator to the L1 position (0).
  public Command ElevatorL1(BooleanSupplier wristLimiter) {

    return MovetoPosition(wristLimiter, -0.6);

  }

  // Command to move the elevator to the L2 position OG(20) New (11.43)
  public Command ElevatorL2(BooleanSupplier wristLimiter) {

    return MovetoPosition(wristLimiter, 12.52);
  }

  // Command to move the elevator to the L3 position OG(52.7). New (30.11)
  public Command ElevatorL3(BooleanSupplier wristLimiter) {

    return MovetoPosition(wristLimiter, 31.2);
  }

  // Command to move the elevator to the L4 position OG (113.7). New(64.97)
  public Command ElevatorL4(BooleanSupplier wristLimiter) {

    return MovetoPosition(wristLimiter, 66).until(() -> elevatorTalonStrb.getForwardLimit().getValueAsDouble() == 1);
  }

  public Command ElevatorA1(BooleanSupplier wristLimiter) {

    return MovetoPosition(wristLimiter, 22.29); // OG (39) New (22.29)
  }

  public Command ElevatorA2(BooleanSupplier wristLimiter) {

    return MovetoPosition(wristLimiter, 40.57); // OG (71) New (40.57)
  }

  public Command ElevatorProcessor(BooleanSupplier wristLimiter) {

    return MovetoPosition(wristLimiter, -0.6);
  }

  public Command ElevatorBarge(BooleanSupplier wristLimiter) {

    return MovetoPosition(wristLimiter, 66.25).until(() -> elevatorTalonStrb.getForwardLimit().getValueAsDouble() == 1); // OG
                                                                                                                         // (114.4)
                                                                                                                         // New
                                                                                                                         // (65.37)
  }

  // Command to exit the current elevator state and maintain its position.
  public Command ExitState(BooleanSupplier wristLimiter) {
    return MovetoPosition(wristLimiter, this.getElevatorPosition());

  }

  // Periodic method called once per scheduler run to update real-time data on
  // Shuffleboard for monitoring.
  @Override
  public void periodic() {

    // checkLimitAndReset();

    // Update the maximum speed value from Shuffleboard.

    // this.maxElevatorSpeed = this.DS_ElevatorSpeed.getDouble(0.2);

    // Update the current position of the elevator.
    this.DS_ElevatorPosition.setDouble(getElevatorPosition());
    if ((elevatorTalonStrb.getReverseLimit().getValueAsDouble() == 1) && (this.getElevatorPosition() != 0)) {
      this.zeroElevator();
    }

    // Update the current status of the forward and reverse limit switches.
    this.DS_forwardLimit.setDouble((this.elevatorTalonStrb.getForwardLimit().getValueAsDouble()));
    this.DS_reverseLimit.setDouble(this.elevatorTalonStrb.getReverseLimit().getValueAsDouble());
    this.DS_ElevatorSetpoint.setDouble(elevatorController.getSetpoint());
    this.DS_canLift.setBoolean(this.canLift);
    SmartDashboard.putData(CommandScheduler.getInstance());
    // The periodic method is called to regularly update the robot's status.
  }

}
