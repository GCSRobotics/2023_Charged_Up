// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystems extends SubsystemBase {
  private CANSparkMax extensionMotor = new CANSparkMax(Constants.EXTENSION_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax elevationMotor = new CANSparkMax(Constants.ELEVATION_MOTOR_ID, MotorType.kBrushless);
  //private VictorSPX elevationMotor = new VictorSPX(Constants.ELEVATION_MOTOR_ID);
  private RelativeEncoder elevationEncoder;
  private RelativeEncoder extensionEncoder;
  private DigitalInput extensionLimit = new DigitalInput(9);

  public static final double HOME_DEGREES = 0;
  public static final double FLOOR_DEGREES = 10;
  public static final double MID_DEGREES = 60;
  public static final double HIGH_DEGREES = 80;
  public static final double HOME_INCHES = 0;
  public static final double FLOOR_INCHES = 10;
  public static final double MID_INCHES = 7;
  public static final double HIGH_INCHES = 26 ;

  /** Creates a new ArmSubsystems. */ 
  public ArmSubsystems() {
    elevationMotor.setInverted(false);
    elevationEncoder = elevationMotor.getEncoder();
    elevationEncoder.setPositionConversionFactor(Constants.ELEVATION_REVOLUTIONS_PER_DEGREE);
    elevationEncoder.setPosition(0);
    elevationMotor.setIdleMode(IdleMode.kBrake);
    
    extensionMotor.setInverted(false);
    extensionEncoder = extensionMotor.getEncoder();
    extensionEncoder.setPositionConversionFactor(Constants.EXTENSION_REVOLUTIONS_PER_INCH);
    extensionEncoder.setPosition(0);
    extensionMotor.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevation Degree", elevationEncoder.getPosition());
    SmartDashboard.putNumber("Extension Position", extensionEncoder.getPosition());
  }

  public void ExtendArm() {
    extensionMotor.set(1.0);
  }

  public void RetractArm() {
    // if (extensionLimit.get()) {
    //   extensionMotor.set(0.0);
    // } else {
      extensionMotor.set(-1.0);
    // }
  }

  public void RaiseArm(double speed) {
    // System.out.println("Position: " + getElevationDegrees() );
    elevationMotor.set(speed);
  }

  public void LowerArm(double speed) {
    // System.out.println("Position: " + getElevationDegrees() );
    elevationMotor.set(speed);
  }

  public void moveArm(double speed) {
    // System.out.println("moveArm");
    elevationMotor.set(speed);
  }

  public void lengthenArm(double speed) {
    // System.out.println("lengthenArm");
    // if (extensionLimit.get() && (speed < 0)) {
    //   extensionMotor.set(0.0);
    // } else {
      extensionMotor.set(speed);
    // }
  }

  public void stopElevation() {
    // elevationMotor.set(0.025);
    elevationMotor.set(0.0);

  }

  public void stopExtension() {
    extensionMotor.set(0.0);
  }

  public double getElevationDegrees() {
    return elevationEncoder.getPosition();
  }

  public void moveArmToDegrees(PIDController pidController, double speed) {
    
    double armposition = getElevationDegrees();
    double output = pidController.calculate(armposition);
    double outputC = MathUtil.clamp(output, -speed, speed);
    // System.out.println("Degrees: " + armposition );

    if (pidController.atSetpoint() || (armposition >= HIGH_DEGREES && outputC > 0) || (armposition < 0 && outputC < 0)) {
      stopElevation();
    } else {
      moveArm(outputC);

    }
  }

  public void extendArmToInches(PIDController pidController, double speed) {
    double armposition = extensionEncoder.getPosition();
    double output = pidController.calculate(armposition);
    double outputC = MathUtil.clamp(output, -speed, speed);
    // System.out.println("Length: " + armposition );
    // System.out.println("Length Setpoint: " + pidController.getSetpoint() );
    // System.out.println("outputC: " + outputC );

    if (pidController.atSetpoint() || (armposition >= HIGH_INCHES && outputC > 0) || (armposition < 0 && outputC < 0)) {
      stopExtension();
    } else {
      lengthenArm(outputC);
    }
  }
}
