// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class UISubsystem extends SubsystemBase {
  /** Creates a new UISubsystem. */
  public UISubsystem() {
    SmartDashboard.putNumber("Max Speed", Constants.Swerve.maxSpeed);
    SmartDashboard.putNumber("Max Angular Velocity", Constants.Swerve.maxAngularVelocity);
  }

  public void checkSpeeds()
  {
    Constants.Swerve.maxSpeed = SmartDashboard.getNumber("Max Speed", 4.5);
    Constants.Swerve.maxAngularVelocity = SmartDashboard.getNumber("Max Angular Velocity", 11.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Max Speed", Constants.Swerve.maxSpeed);
    SmartDashboard.putNumber("Max Angular Velocity", Constants.Swerve.maxAngularVelocity);

    checkSpeeds();
  }


}
