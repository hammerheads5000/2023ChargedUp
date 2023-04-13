// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ChangeSpeedSubsystem;

public class SpeedDownCommand extends CommandBase {
  /** Creates a new SpeedDownCommand. */
  ChangeSpeedSubsystem sub_ChangeSpeedSubsystem;

  public SpeedDownCommand(ChangeSpeedSubsystem sub_ChangeSpeedSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sub_ChangeSpeedSubsystem = sub_ChangeSpeedSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sub_ChangeSpeedSubsystem.SpeedDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sub_ChangeSpeedSubsystem.resetSpeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
