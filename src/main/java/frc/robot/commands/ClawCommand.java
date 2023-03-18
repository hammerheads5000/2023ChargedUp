// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase {
  private final ClawSubsystem clawSubsystem;
  private boolean isFinished = false;
  /** Creates a new ClawCommand. */
  public ClawCommand(ClawSubsystem sub_ClawSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    clawSubsystem = sub_ClawSubsystem;
    addRequirements(clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (clawSubsystem.getState())
    {
      clawSubsystem.m_contract();
      isFinished = true;
      return;
    }
    else if (!clawSubsystem.getState())
    {
      clawSubsystem.m_extend();
      isFinished = true;
      return;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
