// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequential;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.MoveArm;
import frc.robot.commands.Arm.PositionArm;
import frc.robot.subsystems.ArmSubsystems;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetToHigh extends SequentialCommandGroup {
  /** Creates a new SetToHigh. */
  public SetToHigh(ArmSubsystems armSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PositionArm(armSub, ArmSubsystems.FLOOR_INCHES),
      new MoveArm(armSub, ArmSubsystems.HIGH_DEGREES),
      new PositionArm(armSub, ArmSubsystems.HIGH_INCHES)
    );
  }
}
