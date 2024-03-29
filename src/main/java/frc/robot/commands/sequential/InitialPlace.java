// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequential;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Claw.Grab;
import frc.robot.commands.Claw.Release;
import frc.robot.subsystems.ArmSubsystems;
import frc.robot.subsystems.ClawSubsystems;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InitialPlace extends SequentialCommandGroup {
  /** Creates a new InitialPlace. */
  public InitialPlace(ClawSubsystems clawSub, ArmSubsystems armSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Grab(clawSub).withTimeout(.25),
      new SetToHigh(armSub),
      new Release(clawSub).withTimeout(.5),
      new SetToHome(armSub)
    );
  }
}
