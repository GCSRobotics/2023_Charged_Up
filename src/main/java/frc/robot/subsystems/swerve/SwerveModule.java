// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.lib.math.Conversions;  
import frc.lib.util.CTREModuleState;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModule {
    public int moduleNumber;
    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    private final CANCoder angleEncoder;

    private final double angleOffset;
    private double lastAngle;
    public double CANcoderInitTime = 0.0;
    
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveDrivetrain.FF_kS, Constants.SwerveDrivetrain.FF_kV, Constants.SwerveDrivetrain.FF_kA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants swerveModuleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = swerveModuleConstants.angleOffset;

        this.angleEncoder = new CANCoder(swerveModuleConstants.cancoderID);
        this.angleEncoder.configFactoryDefault();
        this.angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCANCoderConfig);

        this.driveMotor = new TalonFX(swerveModuleConstants.driveMotorID);
        this.driveMotor.configFactoryDefault();
        this.driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveTalonFXConfig);
        this.driveMotor.setInverted(Constants.SwerveDrivetrain.DRIVE_MOTOR_INVERTED);
        this.driveMotor.setNeutralMode(Constants.SwerveDrivetrain.DRIVE_NEUTRAL_MODE);
        this.driveMotor.setSelectedSensorPosition(0);

        this.turningMotor = new TalonFX(swerveModuleConstants.angleMotorID);
        this.turningMotor.configFactoryDefault();
        this.turningMotor.configAllSettings(Robot.ctreConfigs.swerveAngleTalonFXConfig);
        this.turningMotor.setInverted(Constants.SwerveDrivetrain.ANGLE_MOTOR_INVERTED);
        this.turningMotor.setNeutralMode(Constants.SwerveDrivetrain.ANGLE_NEUTRAL_MODE);
        
        Timer.delay(0.1);
        waitForCanCoder();
        double absolutePosition = Conversions.degreesToFalcon(this.getCanCoder().getDegrees() - angleOffset, Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO);
        this.turningMotor.setSelectedSensorPosition(absolutePosition);

        this.lastAngle = getState().angle.getDegrees();
    }
    private void waitForCanCoder(){
        /*
         * Wait for CanCoder. (up to 1000ms)
         *
         * preventing race condition during program startup
         */
        CANcoderInitTime = -1;
        for (int i = 0; i < 100; ++i) {
            angleEncoder.getAbsolutePosition();
            if (angleEncoder.getLastError() == ErrorCode.OK) {
                break;
            }
            Timer.delay(0.010);            
            CANcoderInitTime += 10;
        }
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(this.turningMotor.getSelectedSensorPosition(), Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO));
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(this.driveMotor.getSelectedSensorVelocity(), Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO);
        return new SwerveModuleState(velocity, getAngle());
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);    // Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveDrivetrain.MAX_SPEED;
            this.driveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO);
            this.driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveDrivetrain.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle.getDegrees();   // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        this.turningMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO)); 
        this.lastAngle = angle;
    }


    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO), 
            getAngle()
        );
    }

    public void stop() {
        this.turningMotor.set(TalonFXControlMode.PercentOutput, 0);
        this.driveMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
}
