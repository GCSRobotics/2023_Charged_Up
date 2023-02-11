// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleAuton extends SequentialCommandGroup {
  /** Creates a new SimpleAuton. */
  public SimpleAuton(
    SwerveSubsystem swerveSub
  ) {
    TrajectoryConfig config =
    new TrajectoryConfig(
            Constants.Auto.kMaxSpeedMetersPerSecond,
            Constants.Auto.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.SwerveDrivetrain.SWERVE_KINEMATICS);

    Trajectory trajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(4, 0, new Rotation2d(0))),
        config);

        var thetaController =
        new ProfiledPIDController(
            Constants.Auto.kPThetaController, 0, 0, Constants.Auto.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
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
    addCommands(
      new InstantCommand(() -> swerveSub.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveSub.stopModules())

    );
  }
}
