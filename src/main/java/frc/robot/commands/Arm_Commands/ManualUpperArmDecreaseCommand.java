// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.UpperArmManual;

public class ManualUpperArmDecreaseCommand extends CommandBase {

  private final UpperArmManual m_UpperArmManual;

  public ManualUpperArmDecreaseCommand(UpperArmManual m_UpperArmManual) 
  {
    this.m_UpperArmManual = m_UpperArmManual;
    addRequirements(m_UpperArmManual);
  }

  @Override
  public void initialize() 
  {
    
  }

  @Override
  public void execute() 
  {
    m_UpperArmManual.moveDown(.3);
  }

  @Override
  public void end(boolean interrupted) 
  {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
