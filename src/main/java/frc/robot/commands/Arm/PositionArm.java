// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystems;

public class PositionArm extends CommandBase {
  private ArmSubsystems armSubsystems;
  private PIDController pidController = new PIDController(0.1, 0.5, 0);
  private double speed = 1.0;
  private double inchesPosition;

  /** Creates a new PositionArm. */
  public PositionArm(ArmSubsystems armSubsystems, double inchesPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystems = armSubsystems;
    this.inchesPosition = inchesPosition;
    addRequirements(this.armSubsystems);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(inchesPosition);
    pidController.setTolerance(.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystems.extendArmToInches(pidController, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystems.stopExtension();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
