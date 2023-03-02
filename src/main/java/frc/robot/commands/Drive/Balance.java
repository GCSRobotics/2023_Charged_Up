// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class Balance extends CommandBase {
  SwerveSubsystem swerve;
  private final PIDController pidController = new PIDController(0.045, 0, 0.0025);

  /** Creates a new Balance. */
  public Balance(SwerveSubsystem swerve) {
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(0);
    pidController.setTolerance(2.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTip = (Math.abs(swerve.getRoll()) > Math.abs(swerve.getPitch())) ? 
      swerve.getRoll() : swerve.getPitch();
    double output = pidController.calculate(currentTip);

    SmartDashboard.putNumber("pitch", swerve.getRoll()) ;
    SmartDashboard.putNumber("output", output) ;
   
    Translation2d translation = new Translation2d(output, 0).times(Constants.SwerveDrivetrain.MAX_SPEED);
    swerve.drive(translation, 0, true, true);  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
