// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. These constants are used across different
 * subsystems of the robot to avoid hardcoding values multiple times, making
 * the code easier to maintain and modify.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity in the code.
 */
public final class Constants {

  public static final String codeVersion = "Sprint 6.0";

  /**
   * The OperatorConstants class holds constants related to the operator
   * interface (OI), such as controller ports.
   */
  public static class OperatorConstants {
    // Port for the driver controller (gamepad or joystick) connected to the robot
    public static final int kDriverControllerPort = 0;
  }

  /**
   * The ElevatorConstants class contains constants related to the elevator
   * subsystem.
   * This includes limit switch pins and motor controller IDs for controlling
   * the elevator mechanism.
   */
  public static class ElevatorConstants {

    // Pin numbers for high and low limit switches on the elevator
    public static final int HILimitPin = 0;
    public static final int LOLimitPin = 1;

    // CAN IDs for the two motors that control the elevator (elevator port and
    // starboard motors)
    public static final int ElevatorPortMotor_ID = 16;
    public static final int ElevatorStrbMotor_ID = 15;

  }

  /**
   * The EndEffectorsConstants class defines constants for the endEffector
   * subsystem,
   * including CAN IDs for the EndEffector motor and pin assignments for beam
   * break
   * sensors.
   */
  public static class EndEffectorConstants {
    // CAN ID for the motor controlling the endEffector mechanism
    public static final int IntakeCAN_ID = 17;

    // Pin numbers for beam break sensors used to detect objects being picked up or
    // released
    public static final int BeamBreakPinIntake = 2;
    public static final int BeamBreakPinDis = 1;

  }

  /**
   * The WristConstants class holds constants for the wrist subsystem.
   * These include the CAN ID for the wrist motor and encoder.
   */
  public static class WristConstants {

    // CAN ID for the wrist motor that controls wrist movement
    public static final int WristMotor_ID = 10;

    // CAN ID for the wrist encoder (used to measure the position of the wrist)
    public static final int WristCANcoder_ID = 19;

  }

  /**
   * The ClimberConstants class contains constants for the climber subsystem,
   * which may include motors for lifting or extending the robot during climbing,
   * as well as limit switches for detecting positions.
   */
  public static class ClimberConstants {

    // CAN ID for the motor controlling the climber
    public static final int ClimberMotor_ID = 18;

    // CAN ID for the climber encoder (used to measure the climber's position)
    public static final int ClimberCANcoder_ID = 20;

    // Pin numbers for high and low limit switches on the climber
    public static final int HILimitPin = 5;
    public static final int LOLimitPin = 4;

    // Pin number for controlling the intake tray (for raising or lowering
    // it)
    public static final int IntakeTray_Pin = 1;

  }

}
