// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystems extends SubsystemBase {
  private static final DoubleSolenoid CubeSolenoid = new DoubleSolenoid(5, PneumaticsModuleType.REVPH,
      Constants.GrabChannel, Constants.ReleaseChannel);
  private static final DoubleSolenoid ConeSolenoid = new DoubleSolenoid(5, PneumaticsModuleType.REVPH,
      Constants.ConeChannelIn, Constants.ConeChannelOut);

  // private static final DoubleSolenoid FlipSolenoid = new
  // DoubleSolenoid(PneumaticsModuleType.CTREPCM,
  // Constants.FlipUpChannel, Constants.FlipDownChannel);

  private Rev2mDistanceSensor distSensor;

  /** Creates a new ClawSubsystems. */
  public ClawSubsystems() {
    // distSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kInches,
    // RangeProfile.kHighSpeed);
    // distSensor.setAutomaticMode(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (isPiecePresent()) {
    // Grab();
    // }
  }

  public void GrabCube() {
    CubeSolenoid.set(Value.kReverse);
    ConeSolenoid.set(Value.kReverse);
  }

  public void ReleaseCube() {
    CubeSolenoid.set(Value.kForward);
    ConeSolenoid.set(Value.kReverse);
  }

  public void GrabCone() {
    CubeSolenoid.set(Value.kReverse);
    ConeSolenoid.set(Value.kForward);
  }

  public void ReleaseCone() {
    CubeSolenoid.set(Value.kForward);
    ConeSolenoid.set(Value.kReverse);
  }

  // public void FlipUp() {
  // FlipSolenoid.set(Value.kForward);
  // }

  // public void FlipDown(){
  // FlipSolenoid.set(Value.kReverse);
  // }
  public boolean isPiecePresent() {
    return distSensor.isRangeValid() && distSensor.getRange() <= 3.5;
  }
}
