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
  UpperArmSubsystem s_UpperArmSubsystem;

  /** Creates a new UISubsystem. */
  public UISubsystem(LowerArmSubsystem s_LowerArm, ClawSubsystem s_claw, UpperArmSubsystem s_UpperArmSubsystem) {
    this.s_LowerArm = s_LowerArm;
    this.s_UpperArmSubsystem = s_UpperArmSubsystem;
    this.s_claw = s_claw;
  }
  
  @Override
  public void periodic() {
    //Arm
    SmartDashboard.putNumber("Arm Encoder", s_UpperArmSubsystem.getAngle());
    SmartDashboard.putBoolean("Lower arm is up", s_LowerArm.checkState());

    //Claw
    SmartDashboard.putBoolean("Claw is open", s_claw.isOpen);
  }
}
