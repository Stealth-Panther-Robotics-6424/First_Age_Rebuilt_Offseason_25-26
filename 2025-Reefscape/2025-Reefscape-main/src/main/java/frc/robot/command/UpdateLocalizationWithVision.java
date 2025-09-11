// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UpdateLocalizationWithVision extends Command {
  private final Vision vision;
  private final CommandSwerveDrivetrain drivetrain;

  /** Creates a new UpdateLocalizationWithVision. */
  public UpdateLocalizationWithVision(Vision vision, CommandSwerveDrivetrain drivetrain) {
    this.vision = vision;
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    LimelightHelpers.PoseEstimate aftpose = vision.aftLL.getVisionPose();
    LimelightHelpers.PoseEstimate bowpose = vision.bowLL.getVisionPose();
    if (aftpose != null) {
      SmartDashboard.putBoolean("else", false);
      SmartDashboard.putNumber("aftPosX", aftpose.pose.getX());
      SmartDashboard.putNumber("aftPosY", aftpose.pose.getY());
      if (aftpose.rawFiducials[0].ambiguity < 0.7) {
        {

          drivetrain.addVisionMeasurement(aftpose.pose, aftpose.timestampSeconds, VecBuilder.fill(0.1, 0.1, 999999));

        }
      } else {
        SmartDashboard.putBoolean("else", true);
      }
    }
    if (bowpose != null) {
      if (bowpose.rawFiducials[0].ambiguity < 0.7) {
        drivetrain.addVisionMeasurement(bowpose.pose, bowpose.timestampSeconds, VecBuilder.fill(0.1, 0.1, 9999999));
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
