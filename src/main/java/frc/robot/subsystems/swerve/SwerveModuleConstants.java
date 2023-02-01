// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;

    public final int cancoderID;
    public final double angleOffset;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, double angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;

        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
    }
}
