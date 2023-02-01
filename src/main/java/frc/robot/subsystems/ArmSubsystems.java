// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystems extends SubsystemBase {
  private CANSparkMax extensionMotor = new CANSparkMax(Constants.EXTENSION_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax elevationMotor = new CANSparkMax(Constants.ELEVATION_MOTOR_ID, MotorType.kBrushless);

  /** Creates a new ArmSubsystems. */
  public ArmSubsystems() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ExtendArm () {
    extensionMotor.set(1.0);
  }

  public void RetractArm () {
    extensionMotor.set(-1.0);
  }

  public void RaiseArm () {
     elevationMotor.set(1.0);
  }

  public void LowerArm() {
    elevationMotor.set(-1.0);
  }

  public void stopElevation() {
   elevationMotor.set(0.0);
  }

  public void stopExtension() {
  extensionMotor.set(0.0);
  }
}
