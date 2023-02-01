// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystems extends SubsystemBase {
  private static final DoubleSolenoid GrabSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      Constants.GrabChannel, Constants.ReleaseChannel);
 
  private static final DoubleSolenoid FlipSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      Constants.FlipUpChannel, Constants.FlipDownChannel);
        

    
  /** Creates a new ClawSubsystems. */
  public ClawSubsystems() {}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void Grab() {
    GrabSolenoid.set(Value.kForward);
  }
  public void Release() {
    GrabSolenoid.set(Value.kReverse);
  }
  public void FlipUp() {
    FlipSolenoid.set(Value.kForward);
  }

  public void FlipDown(){
    FlipSolenoid.set(Value.kReverse);
  }
}
