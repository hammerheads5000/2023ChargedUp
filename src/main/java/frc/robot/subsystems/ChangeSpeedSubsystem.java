// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChangeSpeedSubsystem extends SubsystemBase {
  /** Creates a new ChangeSpeedSubsystem. */
  public double speed = .5;
  public ChangeSpeedSubsystem() {}

  @Override
  public void periodic() {
  
  }
  public void SpeedUp() {
    speed = Math.min(speed+.25, 1.0);
  }
  public void SpeedDown() {
    speed = Math.max(speed-.25, .25);
  }

  public double getSpeed() {
    return speed;
  }

}
