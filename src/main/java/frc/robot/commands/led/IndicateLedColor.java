// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.RGBColor;
import frc.robot.subsystems.LEDSubsystem;

public class IndicateLedColor extends CommandBase {
    LEDSubsystem ledSub;
    RGBColor gamePieceColor;

  /** Creates a new IndicateCube. */
  public IndicateLedColor(LEDSubsystem subsystem, RGBColor color) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    ledSub = subsystem;
    gamePieceColor = color;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledSub.SetRGB(gamePieceColor.getRed(),gamePieceColor.getGreen(), gamePieceColor.getBlue());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledSub.SetRainbow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
