// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UISubsystem extends SubsystemBase {

  UpperArmManual s_UpperArm;
  LowerArmSubsystem s_LowerArm;
  ClawSubsystem s_claw;

  /** Creates a new UISubsystem. */
  public UISubsystem(UpperArmManual s_UpperArm, LowerArmSubsystem s_LowerArm, ClawSubsystem s_claw) {
    this.s_UpperArm = s_UpperArm;
    this.s_LowerArm = s_LowerArm;
    this.s_claw = s_claw;
    //Swerve module info
  }

  public void UpdateValues() {
    //Arm
    

    // SmartDashboard.putNumber("Upper arm angle", s_UpperArm.);
    SmartDashboard.putBoolean("Lower arm is up", s_LowerArm.checkState());

    //Claw
    SmartDashboard.putBoolean("Claw is open", s_claw.isOpen);

  }

  @Override
  public void periodic() {
  }
}
