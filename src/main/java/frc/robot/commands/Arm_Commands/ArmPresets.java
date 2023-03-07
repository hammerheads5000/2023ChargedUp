// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.UpperArmToSetpoint;


public class ArmPresets extends CommandBase {
  /** Creates a new ArmPresets. */
  UpperArmToSetpoint m_UpperArmToSetpoint;
  public ArmPresets(UpperArmToSetpoint UpperArmToSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_UpperArmToSetpoint = UpperArmToSetpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_UpperArmToSetpoint.MoveArm(40);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
