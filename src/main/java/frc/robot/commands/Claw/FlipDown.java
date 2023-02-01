// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystems;

public class FlipDown extends CommandBase {
private ClawSubsystems clawSubsystems;
  /** Creates a new FlipDown. */
  public FlipDown(ClawSubsystems clawSubsystems) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.clawSubsystems=clawSubsystems;
    addRequirements(this.clawSubsystems);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    clawSubsystems.FlipDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
