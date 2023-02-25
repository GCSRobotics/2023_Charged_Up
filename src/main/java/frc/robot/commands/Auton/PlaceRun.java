// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.MoveArm;
import frc.robot.commands.sequential.PickUpCube;
import frc.robot.commands.sequential.PlaceCubeLow;
import frc.robot.subsystems.ArmSubsystems;
import frc.robot.subsystems.ClawSubsystems;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceRun extends SequentialCommandGroup {
  /** Creates a new PlaceRun. */
  public PlaceRun(
      ArmSubsystems armSubsystem,
      ClawSubsystems clawSub,
      SwerveSubsystem swerveSub,
      PathPlannerTrajectory traj,
      boolean isFirstPath) {

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("place1", new MoveArm(armSubsystem, ArmSubsystems.FLOOR_DEGREES));
    eventMap.put("PickUp", new PickUpCube(clawSub, armSubsystem)); 
    eventMap.put("place2", new PlaceCubeLow(clawSub, armSubsystem));


    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    swerveSub::getPose, // Pose2d supplier
    swerveSub::setPose, // Pose2d consumer, used to reset odometry at the beginning of auto
    Constants.SwerveDrivetrain.SWERVE_KINEMATICS, // SwerveDriveKinematics
    new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    swerveSub::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    swerveSub // The drive subsystem. Used to properly set the requirements of path following commands
);

Command fullAuto = autoBuilder.fullAuto(traj);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            swerveSub.resetOdometry(traj.getInitialHolonomicPose());
          }
        }),
        fullAuto
    );
  }
}
