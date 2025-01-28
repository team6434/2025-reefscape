// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0);
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  /**
   * Converts driver input into field-relative ChassisSpeeds controlled by angular velocity.
   */
  SwerveInputStream driveSwerve = SwerveInputStream
      .of(drivebase.getSwerveDrive(), 
          () -> -driverXbox.getLeftY(),
          () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.DEADBAND)
      .allianceRelativeControl(true);

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    if (RobotBase.isSimulation()) {
      driveSwerve.withControllerRotationAxis(() -> -driverXbox.getRawAxis(2));
    }
    drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveSwerve));
    if (DriverStation.isTest()) {
      driverXbox.a().onTrue(Commands.runOnce(drivebase::zeroGyro));
      driverXbox.b().onTrue(Commands.runOnce(drivebase::centerModulesCommand));
      driverXbox.x().whileTrue(
          drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
    } else {
      driverXbox.a().onTrue(Commands.none());
      driverXbox.b().onTrue(Commands.none());
      driverXbox.x().onTrue(Commands.none());
      driverXbox.y().onTrue(Commands.none());
      driverXbox.back().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
      driverXbox.leftTrigger().onTrue(Commands.none());
      driverXbox.rightTrigger().onTrue(Commands.none());
    }      
    driverXbox.start().onTrue(Commands.runOnce(drivebase::slowDriveUpdate));
    driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
