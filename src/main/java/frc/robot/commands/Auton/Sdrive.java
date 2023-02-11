// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.List;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Sdrive extends SequentialCommandGroup {
  /** Creates a new Sdrive. */
  public Sdrive(SwerveSubsystem swerveSub) {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.Auto.kMaxSpeedMetersPerSecond,
                Constants.Auto.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.SwerveDrivetrain.SWERVE_KINEMATICS);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            Constants.Auto.kPThetaController, 0, 0, Constants.Auto.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            swerveSub::getPose, // Functional interface to feed supplier
            Constants.SwerveDrivetrain.SWERVE_KINEMATICS,

            // Position controllers
            new PIDController(Constants.Auto.kPXController, 0, 0),
            new PIDController(Constants.Auto.kPYController, 0, 0),
            thetaController,
            swerveSub::setModuleStates,
            swerveSub);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // Reset odometry to the starting pose of the trajectory.
    // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> swerveSub.stopModules());
    addCommands(
      new InstantCommand(() -> swerveSub.resetOdometry(exampleTrajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveSub.stopModules())
    );
  }
}
