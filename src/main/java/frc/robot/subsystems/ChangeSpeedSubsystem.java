// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChangeSpeedSubsystem extends SubsystemBase {
  /** Creates a new ChangeSpeedSubsystem. */
  public double speed = .7;
  public ChangeSpeedSubsystem() {}

  @Override
  public void periodic() {
  
  }
  public void SpeedUp() {
    speed = 1.0;
  }
  public void SpeedDown() {
    speed = .3;
  }
  
  public void resetSpeed() {
    speed =.7;
  }
  public double getSpeed() {
    return speed;
  }

}
