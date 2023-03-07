// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LowerArmSubsystem;

public class ManualLowerArmDownCommand extends CommandBase {

  private final LowerArmSubsystem m_LowerArmSubsystem;

  public ManualLowerArmDownCommand(LowerArmSubsystem m_LowerArmSubsystem) 
  {
    this.m_LowerArmSubsystem = m_LowerArmSubsystem;
    addRequirements(m_LowerArmSubsystem);
  }

  @Override
  public void initialize() 
  {
    m_LowerArmSubsystem.m_initializeSolenoid();
    m_LowerArmSubsystem.m_enableCompressor();
  }

  @Override
  public void execute() 
  {
    m_LowerArmSubsystem.m_contract();
  }


  @Override
  public void end(boolean interrupted) 
  {

  }

  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
