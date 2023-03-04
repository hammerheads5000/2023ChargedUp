// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LowerArmSubsystem;

public class LowerArmCommand extends CommandBase {
  private LowerArmSubsystem lowerArmSubsystem;
  private boolean isFinished = false;
  /** Creates a new LowerArmCommand. */
  public LowerArmCommand(LowerArmSubsystem sub_LowerArmSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    lowerArmSubsystem = sub_LowerArmSubsystem;
    addRequirements(lowerArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lowerArmSubsystem.m_initializeSolenoid();
    lowerArmSubsystem.m_enableCompressor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isUp = lowerArmSubsystem.checkState();
    
    if (isUp)
    {
      lowerArmSubsystem.m_contract();
      lowerArmSubsystem.setIsUp(false);
      isFinished = true;
      return;
    }
    else if (!isUp)
    {
      lowerArmSubsystem.m_extend();
      lowerArmSubsystem.setIsUp(true);
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
