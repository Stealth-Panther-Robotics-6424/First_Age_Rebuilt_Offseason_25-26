// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  public final LimelightSubsystem bowLL = new LimelightSubsystem("limelight-bowlime");
  public final LimelightSubsystem aftLL = new LimelightSubsystem("limelight-aft");

  /** Creates a new Vision. */
  public Vision() {

    LimelightHelpers.setCameraPose_RobotSpace("limelight-bowlime", 0.3210213798,
        0.2041934194, 0.2244396832, 0.618624, 15.61692, 3.312392);
    LimelightHelpers.setCameraPose_RobotSpace("limelight-aftlime", 0.1171411424, 0, 0.9671403314, 0, 22, 180);

  }

  public Command TelopVision() {

    return new FunctionalCommand(
        () -> {
        }, // No initialization needed
        () -> {
        }, // Set intake motor power dynamically
        interrupted -> {
        }, // Stop motor if interrupted
        () -> false, // Runs indefinitely
        this // Pass the subsystem
    );

  }

  @Override
  public void periodic() {

  }
  // This method will be called once per scheduler run

}
