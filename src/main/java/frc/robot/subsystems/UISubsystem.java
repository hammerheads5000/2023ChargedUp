// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UISubsystem extends SubsystemBase {

  LowerArmSubsystem s_LowerArm;
  ClawSubsystem s_claw;
  UpperArmToSetpoint s_UpperArmToSetpoint;

  /** Creates a new UISubsystem. */
  public UISubsystem(LowerArmSubsystem s_LowerArm, ClawSubsystem s_claw, UpperArmToSetpoint s_UpperArmToSetpoint) {
    this.s_LowerArm = s_LowerArm;
    this.s_UpperArmToSetpoint = s_UpperArmToSetpoint;
    this.s_claw = s_claw;
  }
  
  @Override
  public void periodic() {
    //Arm
    SmartDashboard.putNumber("Arm Encoder", s_UpperArmToSetpoint.getAngle());
    SmartDashboard.putBoolean("Lower arm is up", s_LowerArm.checkState());

    //Claw
    SmartDashboard.putBoolean("Claw is open", s_claw.isOpen);
  }
}
