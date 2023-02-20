// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveTeleop extends CommandBase {
  /** Creates a new Drive. */
  
  private static final double XDEADBAND = 0.15;
  private static final double YDEADBAND = 0.15;
  private static final double RDEADBAND = 0.2;

  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;
  
  private SwerveSubsystem swerve;
  private CommandJoystick controller;

  public DriveTeleop(SwerveSubsystem swerve, CommandJoystick controller, boolean fieldRelative, boolean openLoop) {
      this.swerve = swerve;
      addRequirements(swerve);

      this.controller = controller;
      this.fieldRelative = fieldRelative;
      this.openLoop = openLoop;
  }

  @Override
  public void execute() {
      // double yAxis = -controller.getLeftY();
      // double xAxis = -controller.getLeftX();
      // double rAxis = -controller.getRightX();

      double yAxis = -controller.getRawAxis(1) * 0.5;
      double xAxis = -controller.getRawAxis(0) * 0.5;
      double rAxis = -controller.getRawAxis(2) * 0.3;
      
      /* Deadbands */
      yAxis = (Math.abs(yAxis) < YDEADBAND) ? 0 : yAxis;
      xAxis = (Math.abs(xAxis) < XDEADBAND) ? 0 : xAxis;
      rAxis = (Math.abs(rAxis) < RDEADBAND) ? 0 : rAxis;

      translation = new Translation2d(yAxis, xAxis).times(Constants.SwerveDrivetrain.MAX_SPEED);
      rotation = rAxis * Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY;
      swerve.drive(translation, rotation, fieldRelative, openLoop);
  }
}