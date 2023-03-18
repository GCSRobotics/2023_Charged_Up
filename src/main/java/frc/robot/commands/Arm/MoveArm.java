// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystems;

public class MoveArm extends CommandBase {
  private ArmSubsystems armSubsystems;
  private PIDController pidController = new PIDController(0.1, 0, 0);
  private double speed = 0.35;
  private double degreePosition;

  /** Creates a new MoveArm. */
  public MoveArm(ArmSubsystems armSubsystems, double degreePosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystems = armSubsystems;
    this.degreePosition = degreePosition;

    addRequirements(this.armSubsystems);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(degreePosition);
    pidController.setTolerance(2.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystems.moveArmToDegrees(pidController, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystems.stopElevation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
