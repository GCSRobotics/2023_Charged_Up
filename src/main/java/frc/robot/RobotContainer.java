// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Arm.ExtendArm;
import frc.robot.commands.Arm.LowerArm;
import frc.robot.commands.Arm.RaiseArm;
import frc.robot.commands.Arm.RetractArm;
import frc.robot.commands.Auton.PlaceRun;
import frc.robot.commands.Auton.Sdrive;
import frc.robot.commands.Claw.FlipDown;
import frc.robot.commands.Claw.FlipUp;
import frc.robot.commands.Claw.Grab;
import frc.robot.commands.Claw.Release;
import frc.robot.commands.Drive.DriveTeleop;
import frc.robot.commands.led.IndicateLedColor;
import frc.robot.commands.sequential.SetToFloor;
import frc.robot.commands.sequential.SetToHigh;
import frc.robot.commands.sequential.SetToHome;
import frc.robot.commands.sequential.SetToMid;
import frc.robot.subsystems.ArmSubsystems;
import frc.robot.subsystems.ClawSubsystems;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
  private final LEDSubsystem ledSub = new LEDSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController opController =
      new CommandXboxController(Constants.OperatorConstants.operatorControllerPort);
  private final CommandJoystick driveController = 
      new CommandJoystick(Constants.OperatorConstants.driverControllerPort);

  private PathPlannerTrajectory placeAndCharge1;
  private PathPlannerTrajectory placeAndCharge2;

  SendableChooser<PathPlannerTrajectory> auton_chooser = new SendableChooser<>();

  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Bring up the default camera server for the RIO camera
    CameraServer.startAutomaticCapture(0);
    DriverStation.silenceJoystickConnectionWarning(true);
    
    setDefaultCommands();
    // Configure the trigger bindings
    configureBindings();
    loadTrajectories();

    led = new AddressableLED(0);
    ledBuffer = new AddressableLEDBuffer(60);
    led.setLength(ledBuffer.getLength());

    led.setData(ledBuffer);
    led.start();
  }

  private void loadTrajectories() {
    auton_chooser.setDefaultOption("Place and Charge 1", PathPlanner.loadPath("Place and Charge 1", new PathConstraints(4, 3)));
    auton_chooser.addOption("Place and Charge 2", PathPlanner.loadPath("Place and Charge 2", new PathConstraints(4, 3)) );

    // Put the chooser on the dashboard
    SmartDashboard.putData(auton_chooser);

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
    opController.y().onTrue(new SetToHigh(armSub));
    opController.a().onTrue(new SetToFloor(armSub));
    opController.b().onTrue(new SetToMid(armSub));
    opController.x().onTrue(new SetToHome(armSub));
    opController.povUp().whileTrue(new RaiseArm(armSub));
    opController.povDown().whileTrue(new LowerArm(armSub));
    opController.povRight().whileTrue(new ExtendArm(armSub));
    opController.povLeft().whileTrue(new RetractArm(armSub));
    opController.leftTrigger().onTrue(new IndicateLedColor(ledSub, Constants.CubeColor).withTimeout(5));
    opController.rightTrigger().onTrue(new IndicateLedColor(ledSub, Constants.ConeColor).withTimeout(5));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    PathPlannerTrajectory trajectory = auton_chooser.getSelected();
  
    return new PlaceRun(armSub, clawSub, swerveSub, trajectory, false);

  }
}
