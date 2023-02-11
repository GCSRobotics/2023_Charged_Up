// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Arm.ExtendArm;
import frc.robot.commands.Arm.LowerArm;
import frc.robot.commands.Arm.RaiseArm;
import frc.robot.commands.Arm.RetractArm;
import frc.robot.commands.Auton.Sdrive;
import frc.robot.commands.Claw.FlipDown;
import frc.robot.commands.Claw.FlipUp;
import frc.robot.commands.Claw.Grab;
import frc.robot.commands.Claw.Release;
import frc.robot.commands.Drive.DriveTeleop;
import frc.robot.subsystems.ArmSubsystems;
import frc.robot.subsystems.ClawSubsystems;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ArmSubsystems armSub = new ArmSubsystems();
  private final ClawSubsystems clawSub = new ClawSubsystems();
  private final SwerveSubsystem swerveSub = new SwerveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController opController =
      new CommandXboxController(Constants.OperatorConstants.operatorControllerPort);
  private final CommandJoystick driveController = 
      new CommandJoystick(Constants.OperatorConstants.driverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    
    setDefaultCommands();
    // Configure the trigger bindings
    configureBindings();
  }

  private void setDefaultCommands() {
    swerveSub.setDefaultCommand(new DriveTeleop(swerveSub, driveController, true, true));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    opController.y().onTrue(new FlipUp(clawSub));
    opController.a().onTrue(new FlipDown(clawSub));
    opController.b().onTrue(new Grab(clawSub));
    opController.x().onTrue(new Release(clawSub));
    opController.povUp().whileTrue(new RaiseArm(armSub));
    opController.povDown().whileTrue(new LowerArm(armSub));
    opController.povRight().whileTrue(new ExtendArm(armSub));
    opController.povLeft().whileTrue(new RetractArm(armSub));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new Sdrive(swerveSub);
  }
}
