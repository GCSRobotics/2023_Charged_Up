// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.LowerArm;
import frc.robot.commands.Arm.RaiseArm;
import frc.robot.subsystems.ArmSubsystems;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceRun extends SequentialCommandGroup {
  /** Creates a new PlaceRun. */
  public PlaceRun(
      ArmSubsystems armSubsystem,
      SwerveSubsystem swerveSub,
      PathPlannerTrajectory traj,
      boolean isFirstPath) {

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    eventMap.put("intakeDown", new RaiseArm(armSubsystem));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            swerveSub.resetOdometry(traj.getInitialHolonomicPose());
          }
        }),
        new RaiseArm(armSubsystem),
        new LowerArm(armSubsystem),
        new PPSwerveControllerCommand(
            traj,
            swerveSub::getPose, // Pose supplier
            Constants.SwerveDrivetrain.SWERVE_KINEMATICS, // SwerveDriveKinematics
            new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use
                                        // feedforwards.
            new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will
                                        // only use feedforwards.
            swerveSub::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color.
                  // Optional, defaults to true
            swerveSub // Requires this drive subsystem
        ));
  }
}
