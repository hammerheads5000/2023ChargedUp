// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ChangeSpeedSubsystem extends SubsystemBase {
  /** Creates a new ChangeSpeedSubsystem. */
  double RegularSpeed = Constants.Swerve.RegularSpeedMultiplier;
  double speed = RegularSpeed;
  boolean SpeedyQuick;
  public ChangeSpeedSubsystem() {}

  @Override
  public void periodic() {
  
  }
  public void SpeedUp() {
    speed = Constants.Swerve.MaxSpeedMultiplier;
  }
  public void SpeedDown() {
    speed = Constants.Swerve.MinspeedMultiplier;
  }
  
  public void resetSpeed() {
    speed = RegularSpeed;
  }

  public void ToggleRegularSpeed()
  {
    if(SpeedyQuick) {
      RegularSpeed =  Constants.Swerve.RegularSpeedMultiplier;
    }
    else {
      RegularSpeed = Constants.Swerve.LowRegularSpeedMultiplier;
    }
    SpeedyQuick = !SpeedyQuick;
  }
  public double getSpeed() {
    return speed;
  }

}
