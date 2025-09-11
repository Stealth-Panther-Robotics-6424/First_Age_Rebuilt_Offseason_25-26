// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*TODO 
 *  Base Sprint 1/27/2025
 *  Base Sprint 3.0 
 *  Base Sprint 4.0
 *  Base Sprint 5.0
*/

// Import necessary packages for robot control, mathematical operations, and subsystem handling
package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.reflect.Method;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.command.UpdateLocalizationWithVision;
import frc.robot.command.MergeVisionOdometryCommand;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Vision;

@SuppressWarnings("unused")
public class RobotContainer {

        private ShuffleboardTab DS_MainTab = Shuffleboard.getTab("Main");
        private GenericEntry DS_CodeVersion = DS_MainTab.add("Code Version", Constants.codeVersion).getEntry();
        private GenericEntry DS_AlgeaMode = DS_MainTab.add("Algea Mode", false).getEntry();
        private GenericEntry DS_DriveMode = DS_MainTab.add("Drive Mode", false).getEntry();

        // Define maximum speed and angular rate based on tuner constants, converted
        // into appropriate units
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        // Create swerve drive requests for field-centric and robot-centric driving
        // modes with deadband
        private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // Set up brake and wheel pointing functionalities for swerve drive
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        /* Path follower for autonomous control */
        // private SendableChooser<String> autoChooser;
        private SendableChooser<Command> autoChooser;
        // Create a telemetry logger for monitoring robot stats
        // private final Telemetry logger = new Telemetry(MaxSpeed);

        // Joysticks for controlling the robot, one for driving and one for the button
        // board
        private final CommandJoystick joystick = new CommandJoystick(0); // My joystick
        private final CommandJoystick buttonbord = new CommandJoystick(1); // Buttonboard joystick
        private final Wrist wrist; // Wrist subsystem for arm control
        private final Elevator elevator; // Elevator subsystem for vertical movements
        private final EndEffector endEffector; // Intake subsystem for grabbing objects
        private final Climber climber; // Intake subsystem for grabbing objects
        private final Vision vision; // Intake subsystem for grabbing objects

        private boolean AlgeaMode = false;

        // Instantiate the swerve drivetrain subsystem
        public final CommandSwerveDrivetrain drivetrain;

        // Define triggers for controlling wrist and elevator limits
        private final Trigger wristLimiter;
        private final Trigger canFold;
        private final Trigger wristIntake;
        private final Trigger elevatorIntake;
        private final Trigger isEnabled;
        private final Trigger isDisabled;
        private final Trigger algeaModeEnabled;
        private final Trigger reverseLimitHit;
        private final Trigger BBLockout;
        private final Trigger DontIntakeWrist;

        /* Some triggers related to elevator throttles (to be developed in Sprint 4) */
        /*
         * private final Trigger driveThrottleL2 = elevator.driveThrottleL2(); TODO
         * Sprint 4 beta
         * private final Trigger driveThrottleL3 = elevator.driveThrottleL3();
         * private final Trigger driveThrottleL4 = elevator.driveThrottleL4();
         * 
         * 
         * 
         * /*
         * 
         * Class Constructor
         * 
         */
        public RobotContainer() {
                // Set up autonomous command chooser using PathPlanner
                drivetrain = TunerConstants.createDrivetrain();
                wrist = new Wrist();
                elevator = new Elevator(); // Elevator subsystem for vertical movements
                endEffector = new EndEffector(); // Intake subsystem for grabbing objects
                climber = new Climber(); // Intake subsystem for grabbing objects
                vision = new Vision();

                wristLimiter = wrist.wristLimiter();
                canFold = elevator.canFold();
                BBLockout = endEffector.BBLockout();
                wristIntake = wrist.wristIntake();
                elevatorIntake = elevator.elevatorIntake();
                isEnabled = new Trigger(() -> DriverStation.isEnabled());
                isDisabled = new Trigger(() -> DriverStation.isDisabled());
                algeaModeEnabled = new Trigger(() -> getAlgeaMode());
                reverseLimitHit = elevator.reverseLimitHit();
                DontIntakeWrist = wrist.wristDontIntake();
                // selector on the dashboard

                // Define and register commands for the intake subsystem with different
                // behaviors

                configureBindings(); // Configure control bindings for robot functions

                NamedCommands.registerCommand("Intake Coral", endEffector.IntakeCoral());
                NamedCommands.registerCommand("Stop Intake", endEffector.nothing().withTimeout(0.1));
                NamedCommands.registerCommand("Shoot Coral", endEffector.shootCoral().withTimeout(.25));
                NamedCommands.registerCommand("Shoot Algea", endEffector.ShootAlgea().withTimeout(2));
                NamedCommands.registerCommand("Intake Algea", endEffector.IntakeAlgea().withTimeout(.5));
                NamedCommands.registerCommand("Hold Algea", endEffector.HoldAlgea());

                new EventTrigger("L4").onTrue((Commands.sequence(wrist.WristSafety(
                                () -> canFold.getAsBoolean()), elevator.ElevatorL4(wristLimiter),
                                wrist.WristL4(() -> canFold.getAsBoolean()))));
                new EventTrigger("L1").onTrue(Commands.sequence(wrist.WristSafety(
                                () -> canFold.getAsBoolean()), elevator.ElevatorL1(wristLimiter),
                                wrist.WristL1(() -> canFold.getAsBoolean())));
                new EventTrigger("A1").onTrue(Commands.sequence(wrist.WristSafety(
                                () -> canFold.getAsBoolean()), elevator.ElevatorA1(wristLimiter),
                                wrist.WristA1(() -> canFold.getAsBoolean())));

                new EventTrigger("Processor").onTrue(Commands.sequence(wrist.WristSafety(
                                () -> canFold.getAsBoolean()),
                                elevator.ElevatorProcessor(wristLimiter),
                                wrist.WristProcessor(() -> canFold
                                                .getAsBoolean())));

                /*
                 * NamedCommands.registerCommand("Wrist Safety", wrist.WristSafety(canFold));
                 * NamedCommands.registerCommand("L4 Elevator",
                 * elevator.ElevatorL4(wristLimiter));
                 * NamedCommands.registerCommand("L4 Wrist", wrist.WristL4(canFold));
                 * 
                 * 
                 * NamedCommands.registerCommand("L4 Command",
                 * (Commands.sequence(wrist.WristSafety(
                 * () -> canFold.getAsBoolean()), elevator.ElevatorL4(wristLimiter),
                 * wrist.WristL4(() -> canFold.getAsBoolean()))));
                 * 
                 * 
                 */

                // autoChooser = new SendableChooser<String>();
                // autoChooser.addOption("test auto", "test auto");
                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("Auto Mode", autoChooser); // Display auto mode
        }

        public boolean getAlgeaMode() { // Sets whether the commands are based on algea or coral
                return this.AlgeaMode;

        }

        public void DS_Update() {
                this.DS_AlgeaMode.setBoolean(this.getAlgeaMode()); // updates the current mode of the robot

        }

        // Configure the control bindings for the robot's subsystems and commands
        private void configureBindings() {

                buttonbord.button(8).onTrue(Commands.runOnce(() -> this.AlgeaMode = !this.AlgeaMode));

                vision.setDefaultCommand(new MergeVisionOdometryCommand(vision, drivetrain));

                // Drivetrain control for swerve drive based on joystick input
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> fieldCentricDrive
                                                .withVelocityX(-joystick.getRawAxis(1) * MaxSpeed * throttle(3)
                                                                * elevator.elevatorThrottle()) // Drive
                                                                                               // forward
                                                                                               // with
                                                                                               // negative
                                                                                               // Y
                                                                                               // (forward)
                                                .withVelocityY(-joystick.getRawAxis(0) * MaxSpeed * throttle(3)
                                                                * elevator.elevatorThrottle()) // Drive
                                                                                               // left
                                                                                               // with
                                                                                               // negative
                                                                                               // X
                                                                                               // (left)
                                                .withRotationalRate(
                                                                -joystick.getRawAxis(4) * MaxAngularRate * throttle(3)
                                                                                * elevator.elevatorThrottle()) // Drive
                                                                                                               // counterclockwise
                                                                                                               // with
                                                                                                               // negative
                                                                                                               // X
                                                                                                               // (left)
                                ));

                // Button bindings for switching to robot-centric control mode
                joystick.button(8).toggleOnTrue(drivetrain.applyRequest(() -> robotCentricDrive
                                .withVelocityX(-joystick.getRawAxis(1) * MaxSpeed * throttle(3)
                                                * elevator.elevatorThrottle()) // Drive forward with
                                // negative Y (forward)
                                .withVelocityY(-joystick.getRawAxis(0) * MaxSpeed * throttle(3)
                                                * elevator.elevatorThrottle()) // Drive left with
                                                                               // negative X (left)
                                .withRotationalRate(-joystick.getRawAxis(4) * MaxAngularRate * throttle(3)
                                                * elevator.elevatorThrottle())) // Drive
                                                                                // counterclockwise
                                                                                // with
                                                                                // negative
                                                                                // X (left)
                );

                // Joystick button to reset field-centric heading
                joystick.button(3).and(joystick.button(2))
                                .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // Joystick button to apply the brake to stop all swerve drive modules
                joystick.button(4).whileTrue(drivetrain.applyRequest(() -> brake));

                // Button to point the wheels in a specific direction based on joystick input
                /*
                 * joystick.button(3).whileTrue(drivetrain.applyRequest(
                 * () -> point.withModuleDirection(
                 * new Rotation2d(-joystick.getRawAxis(1), -joystick.getRawAxis(0)))));
                 */

                // Run SysId routines when specific button combinations are pressed
                /*
                 * joystick.button(7).and(joystick.button(4)).whileTrue(drivetrain.sysIdDynamic(
                 * Direction.kForward));
                 * joystick.button(7).and(joystick.button(1)).whileTrue(drivetrain.sysIdDynamic(
                 * Direction.kReverse));
                 * joystick.button(8).and(joystick.button(4)).whileTrue(drivetrain.
                 * sysIdQuasistatic(Direction.kForward));
                 * joystick.button(8).and(joystick.button(1)).whileTrue(drivetrain.
                 * sysIdQuasistatic(Direction.kReverse));
                 */
                // Register telemetry logging for the drivetrain subsystem
                // drivetrain.registerTelemetry(logger::telemeterize);

                // Endeffector command bindings, such as when to turn on intake or control trays
                endEffector.setDefaultCommand(endEffector.nothing()); // Default is do nothing
                // Coral Commands
                algeaModeEnabled.negate().and((((elevator.elevatorIntake().and(wrist.wristIntake())))
                                .and(() -> RobotState.isTeleop())))
                                .onTrue(Commands.sequence(endEffector.Intake(), endEffector.Hold(),
                                                endEffector.FeedForward(), endEffector.FeedBack(),
                                                endEffector.FeedForward(),
                                                endEffector.TeleIntakeCoral(DontIntakeWrist)));// When algea
                // mode is
                // disabled
                // and the
                // elevator
                // and wrist are in the L1 position

                (buttonbord.button(5))
                                .onTrue(Commands.sequence(endEffector.Intake(), endEffector.Hold(),
                                                endEffector.FeedForward(), endEffector.FeedBack(),
                                                endEffector.FeedForward(),
                                                endEffector.TeleIntakeCoral(DontIntakeWrist)));

                // intake coral

                algeaModeEnabled.negate().and(DontIntakeWrist.negate()).and(() -> RobotState
                                .isTeleop()) // .and(buttonbord.button(5))
                                .whileTrue(endEffector.IntakeCoral());
                // When algea mode is diabled and button 5 is hit Intake coral manually
                algeaModeEnabled.negate().and(buttonbord.button(2)).whileTrue(endEffector.shootCoral());
                // When algea mode is diabled and button 2 is hit shoot coral
                algeaModeEnabled.negate().and(buttonbord.button(3)).whileTrue(endEffector.manualBackFeed());
                // When algea mode is diabled and button 3 is hit backfeed coral

                // Algea Commands
                algeaModeEnabled.and(buttonbord.button(5)).whileTrue(endEffector.IntakeAlgea());
                // When algea mode is enabled and button 5 is hit Intake algea
                algeaModeEnabled.and(buttonbord.button(5).negate()).and(buttonbord.button(2)
                                .negate())
                                .whileTrue(endEffector.HoldAlgea());
                // When algea mode is enabled and button 2 and button 5 are not hit hold algea
                // by applying a 3% back spin

                algeaModeEnabled.and(buttonbord.button(2)).and(buttonbord.button(5).negate())
                                .whileTrue(endEffector.ShootAlgea());
                // When algea mode is enabled and button 2 is hit and button 5 is not hit shoot
                // algea

                // Commands for wrist and elevator control using buttonboard inputs
                wrist.setDefaultCommand(wrist.WristPIDCommandDefault(() -> canFold.getAsBoolean()));
                buttonbord.axisGreaterThan(0, 0.1).or(buttonbord.axisLessThan(0, -0.1))
                                .whileTrue(wrist.ManualWrist(() -> buttonbord.getRawAxis(0),
                                                () -> canFold.getAsBoolean()));

                // Elevator control with wrist limit consideration
                elevator.setDefaultCommand((elevator.elevatorPIDCommandDefault(() -> wristLimiter.getAsBoolean())));
                (wristLimiter).and((buttonbord.axisGreaterThan(1, 0.1).or(buttonbord.axisLessThan(1, -0.1)))
                                .whileTrue(elevator.ManualElevator(() -> buttonbord.getRawAxis(1),
                                                () -> wristLimiter.getAsBoolean())));

                // Commands to preserve position when enabled
                this.isEnabled.onTrue(elevator.startCommand(wristLimiter));
                this.isEnabled.onTrue(wrist.startWristCommand());
                // this.isDisabled.onTrue(elevator.EndCommand(wristLimiter));

                // Wrist and elevator commands for specific positions, triggered by button
                // presses
                (BBLockout.negate()).and(algeaModeEnabled.negate().and(buttonbord.button(1)))
                                .onTrue(Commands.sequence(wrist.WristSafety(
                                                () -> canFold.getAsBoolean()), elevator.ElevatorL4(wristLimiter),
                                                wrist.WristL4(() -> canFold.getAsBoolean())));
                (BBLockout.negate()).and(algeaModeEnabled.negate().and(buttonbord.button(4)))
                                .onTrue(Commands.sequence(wrist.WristSafety(
                                                () -> canFold.getAsBoolean()), elevator.ElevatorL3(wristLimiter),
                                                wrist.WristL3(() -> canFold.getAsBoolean())));
                (BBLockout.negate()).and(algeaModeEnabled.negate().and(buttonbord.button(7)))
                                .onTrue(Commands.sequence(wrist.WristSafety(
                                                () -> canFold.getAsBoolean()), elevator.ElevatorL2(wristLimiter),
                                                wrist.WristL2(() -> canFold.getAsBoolean())));
                (BBLockout.negate()).and(algeaModeEnabled.negate().and(buttonbord.button(11)))
                                .onTrue(Commands.sequence(wrist.WristSafety(
                                                () -> canFold.getAsBoolean()), elevator.ElevatorL1(wristLimiter),
                                                wrist.WristL1(() -> canFold.getAsBoolean())));
                // Algea Positions//
                (algeaModeEnabled.and(buttonbord.button(7)))
                                .onTrue(Commands.sequence(wrist.WristSafety(
                                                () -> canFold.getAsBoolean()), elevator.ElevatorA1(wristLimiter),
                                                wrist.WristA1(() -> canFold.getAsBoolean())));

                (algeaModeEnabled.and(buttonbord.button(4)))
                                .onTrue(Commands.sequence(wrist.WristSafety(
                                                () -> canFold.getAsBoolean()),
                                                elevator.ElevatorA2(wristLimiter),
                                                wrist.WristA2(() -> canFold
                                                                .getAsBoolean())));

                (algeaModeEnabled.and(buttonbord.button(1)))
                                .onTrue(Commands.sequence(wrist.WristSafety(
                                                () -> canFold.getAsBoolean()), elevator.ElevatorBarge(wristLimiter),
                                                wrist.WristBarge(() -> canFold.getAsBoolean())));

                (algeaModeEnabled.and(buttonbord.button(11)))
                                .onTrue(Commands.sequence(wrist.WristSafety(
                                                () -> canFold.getAsBoolean()),
                                                elevator.ElevatorProcessor(wristLimiter),
                                                wrist.WristProcessor(() -> canFold
                                                                .getAsBoolean())));

                /*
                 * algeaModeEnabled.and(buttonbord.button(9))
                 * .onTrue(Commands.sequence(wrist.WristSafety(
                 * () -> canFold.getAsBoolean()),
                 * elevator.ElevatorProcessor(wristLimiter),
                 * wrist.WristProcessor(() -> canFold
                 * .getAsBoolean())));
                 */

                joystick.button(5).or(joystick.button(6))
                                .onTrue(climber.ManualClimber(() -> joystick.button(6).getAsBoolean(),
                                                () -> joystick.button(5).getAsBoolean()));

                RobotModeTriggers.teleop().whileTrue(vision.TelopVision());

                // (reverseLimitHit).onTrue(elevator.Zero());

                // Buttonboard button 8 toggles manual tray control for the intake

                (BBLockout.negate()).and(buttonbord.button(12)).onTrue(Commands.sequence(wrist.WristSafety(
                                () -> canFold.getAsBoolean()),
                                elevator.ElevatorL1(wristLimiter),
                                wrist.WristClimber(() -> canFold
                                                .getAsBoolean())));

                buttonbord.button(6)
                                .toggleOnTrue(climber.TrayManualUp());

                joystick.button(7).toggleOnTrue(climber.TrayManualDown());

        }

        private double throttle(int throttle_axis) {
                return ((1 - (0.8 * Math.pow(this.joystick.getRawAxis(throttle_axis), 0.5))));
        }

        // Autonomous command that is selected based on the chosen auto mode
        public Command getAutonomousCommand() {
                /* Run the path selected from the auto chooser */
                // return new PathPlannerAuto(autoChooser.getSelected());
                return autoChooser.getSelected();
        }
}