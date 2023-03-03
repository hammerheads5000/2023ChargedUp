// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmToSetpoint;

public class ArmAtLimit extends CommandBase {
  /** Creates a new ArmAtLimit. */
  private ArmToSetpoint m_UpperArmToSetpoint;
  private ArmToSetpoint m_LowerArmToSetpoint;
  private DigitalInput upperLowerSwitch;
  private DigitalInput upperUpperSwitch;
  private DigitalInput lowerLowerSwitch;
  private DigitalInput lowerUpperSwitch;
  
  public ArmAtLimit(ArmToSetpoint UpperarmToSetpoint, ArmToSetpoint LowerArmToSetPoint, DigitalInput one, DigitalInput two, DigitalInput three, DigitalInput four) 
  {
    m_LowerArmToSetpoint = LowerArmToSetPoint;
    m_UpperArmToSetpoint = UpperarmToSetpoint;
    upperLowerSwitch = one;
    upperUpperSwitch = two;
    lowerLowerSwitch = three;
    lowerUpperSwitch = four;
    addRequirements(UpperarmToSetpoint);
    addRequirements(LowerArmToSetPoint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
  
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
