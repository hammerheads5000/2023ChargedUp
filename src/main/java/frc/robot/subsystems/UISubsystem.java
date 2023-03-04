// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UISubsystem extends SubsystemBase {
  ClawSubsystem s_claw;
  IntakeSubsystem s_intake;



  /** Creates a new UISubsystem. */
  public UISubsystem( ClawSubsystem s_claw, IntakeSubsystem s_intake) {
    this.s_claw = s_claw;
    this.s_intake = s_intake;
  }

  public void UpdateValues() {
    //Arm
    //Claw
    SmartDashboard.putBoolean("Claw is open", s_claw.isOpen);

    //Intake
    SmartDashboard.putBoolean("Intake is open", s_intake.isGrabberOpen);
  }

  @Override
  public void periodic() {
  }
}
