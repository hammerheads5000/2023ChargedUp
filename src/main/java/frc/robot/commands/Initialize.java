// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LowerArmSubsystem;
import frc.robot.subsystems.UpperArmToSetpoint;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Initialize extends InstantCommand {
  UpperArmToSetpoint s_arm;
  LowerArmSubsystem s_lowerArm;
  ClawSubsystem s_claw;
  public Initialize(UpperArmToSetpoint s_arm, LowerArmSubsystem s_lowerArm, ClawSubsystem s_claw) {
    this.s_arm = s_arm;
    this.s_lowerArm = s_lowerArm;
    this.s_claw = s_claw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_arm.MoveArmPath(ArmConstants.resting, s_lowerArm);
    s_claw.m_contract();
  }
}
