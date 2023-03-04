// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UISubsystem extends SubsystemBase {
  UpperArmToSetpoint s_UpperArm;
  LowerArmSubsystem s_LowerArm;
  EncoderCheck s_encoderCheck;
  ClawSubsystem s_claw;
  IntakeSubsystem s_intake;



  /** Creates a new UISubsystem. */
  public UISubsystem(UpperArmToSetpoint s_UpperArm, LowerArmSubsystem s_LowerArm, EncoderCheck s_encoderCheck, ClawSubsystem s_claw, IntakeSubsystem s_intake) {
    this.s_UpperArm = s_UpperArm;
    this.s_LowerArm = s_LowerArm;
    this.s_encoderCheck = s_encoderCheck;
    this.s_claw = s_claw;
    this.s_intake = s_intake;
  }

  public void UpdateValues() {
    //Arm
    SmartDashboard.putBoolean("Lower arm is back", s_encoderCheck.Lower_ArmBackwardsSwitch.get());
    SmartDashboard.putBoolean("Lower arm is forward", s_encoderCheck.Lower_ArmForwardsSwitch.get());
    SmartDashboard.putBoolean("Upper arm will break height limit if lower arm is forward", s_encoderCheck.Upper_MaxWhileForwardsSwitch.get());
    SmartDashboard.putBoolean("Upper arm will break height limit if lower arm is backward", s_encoderCheck.Upper_MaxWhileBackwardsSwitch.get());
    SmartDashboard.putBoolean("Safe to bring lower arm back w/o breaking height limit", s_encoderCheck.Upper_BringArmUpSafetySwitch.get());
    SmartDashboard.putBoolean("Upper arm is at goalpost if lower arm is forawrd", s_encoderCheck.Upper_AtStowSwitch.get());
    SmartDashboard.putNumber("Upper arm angle", s_UpperArm.angle);
    SmartDashboard.putBoolean("Lower arm is up", s_LowerArm.checkState());

    //Claw
    SmartDashboard.putBoolean("Claw is open", s_claw.isOpen);

    //Intake
    SmartDashboard.putBoolean("Intake is open", s_intake.isGrabberOpen);
  }

  @Override
  public void periodic() {
  }
}
