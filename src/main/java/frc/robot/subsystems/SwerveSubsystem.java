// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] swerveModules;
  private Pigeon2 gyro;
  private Field2d field;

  /** Creates a new Swerve. */
  public SwerveSubsystem() {
    this.gyro = new Pigeon2(Constants.SwerveDrivetrain.GYRO_ID);
    this.gyro.configFactoryDefault();
    this.zeroGyro();

    this.swerveModules = new SwerveModule[] {
      new SwerveModule(0, Constants.SwerveDrivetrain.Mod0.constants),
      new SwerveModule(1, Constants.SwerveDrivetrain.Mod1.constants),
      new SwerveModule(2, Constants.SwerveDrivetrain.Mod2.constants),
      new SwerveModule(3, Constants.SwerveDrivetrain.Mod3.constants)
    };

    this.swerveOdometry = new SwerveDriveOdometry(Constants.SwerveDrivetrain.SWERVE_KINEMATICS, this.getYaw(), getModulePositions());

    this.field = new Field2d();

    dashboard();
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.SwerveDrivetrain.SWERVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative ?
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                this.getYaw()
            )
        :
            new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation
            )
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveDrivetrain.MAX_SPEED);

    for (SwerveModule mod : this.swerveModules) {
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Front Left Module Angle", this.swerveModules[0].getCanCoder().getDegrees());
    SmartDashboard.putNumber("Front Right Module Angle", this.swerveModules[1].getCanCoder().getDegrees());
    SmartDashboard.putNumber("Back Right Module Angle", this.swerveModules[2].getCanCoder().getDegrees());
    SmartDashboard.putNumber("Back Left Module Angle", this.swerveModules[3].getCanCoder().getDegrees());

  }

    /* Gyro */
    public void zeroGyro() {
      this.gyro.setYaw(0);
  }


  private double optimizeGyro (double degrees) {
    // 0 < degrees < 360
    if ((degrees > 0.0) && (degrees < 360.0)) {
        return degrees;
    } else {
        int m = (int) Math.floor( degrees / 360.0 );
        double optimizedDegrees = degrees - (m * 360.0);
        return Math.abs(optimizedDegrees);
    }
  }

  public Rotation2d getYaw() {
      double[] ypr = new double[3];
      this.gyro.getYawPitchRoll(ypr);
      double yaw = optimizeGyro(ypr[0]);
      return Constants.SwerveDrivetrain.INVERT_GYRO ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
  }

  public double getGyroAngleDegrees() {
      return this.getYaw().getDegrees();
  }

  public double getGyroAngleRadians() {
      return this.getYaw().getRadians();
  }

  /* Odometry */
  public Pose2d getPose() {
      return this.swerveOdometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
      this.swerveOdometry.resetPosition(pose.getRotation(), getModulePositions(), pose);
  }

  public void resetOdometry(Pose2d pose) {
      this.swerveOdometry.resetPosition(this.getYaw(), getModulePositions(), pose);
  }

  public void dashboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    tab.add(this);
    tab.addNumber("Gyro Angle ???", this::getGyroAngleDegrees).withWidget(BuiltInWidgets.kGyro);
    tab.addNumber("Gyro Angle (GRAPH) ???", this::getGyroAngleDegrees).withWidget(BuiltInWidgets.kGraph);
    SmartDashboard.putData(this.field);
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    for(SwerveModule mod : swerveModules){
        states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions(){
      SwerveModulePosition[] positions = new SwerveModulePosition[4];
      for(SwerveModule mod : swerveModules){
          positions[mod.moduleNumber] = mod.getPosition();
      }
      return positions;
  }

}
