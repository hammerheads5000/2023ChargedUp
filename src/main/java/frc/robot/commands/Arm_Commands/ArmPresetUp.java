// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm_Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LowerArmSubsystem;
import frc.robot.subsystems.UpperArmToSetpoint;
import frc.robot.ArmPreset;
import frc.robot.Constants.ArmConstants; 

public class ArmPresetUp extends CommandBase {
  /** Creates a new ArmPresets. */
  UpperArmToSetpoint sub_UpperArmToSetpoint;
  LowerArmSubsystem sub_LowerArmSubsystem;

  public ArmPresetUp(UpperArmToSetpoint UpperArmToSetpoint, LowerArmSubsystem sub_LowerArmSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    sub_UpperArmToSetpoint = UpperArmToSetpoint;
    this.sub_LowerArmSubsystem = sub_LowerArmSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int current = currentPreset();
    int newIndex = (current == -1) ? 0 : Math.min(current+1, ArmConstants.presets.length-1);
    ArmPreset desired = ArmConstants.presets[newIndex];
    sub_UpperArmToSetpoint.SetArm(desired.getAngle());
    sub_LowerArmSubsystem.setIsUp(desired.getLowerArmUp());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
  }
  
  // returns index of current preset in presets array, -1 if not found
  private int currentPreset() {
    ArmPreset measured = new ArmPreset(sub_UpperArmToSetpoint.getAngle(), sub_LowerArmSubsystem.checkState());
    for (int i = 0; i < ArmConstants.presets.length; i++) {
      if (measured.equals(ArmConstants.presets[i])) {
        return i;
      }
    }
    return -1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  private void MoveArmPath(ArmPreset desired)
  {
    if(sub_LowerArmSubsystem.checkState() == desired.getLowerArmUp())
    {
      sub_UpperArmToSetpoint.SetArm(desired.getAngle());
    }
    else 
    {
      if(sub_LowerArmSubsystem.checkState())
      {
        if((desired.getAngle() > ArmConstants.MaxAngleWhileUp))
        {
          sub_UpperArmToSetpoint.SetArm(ArmConstants.MinAngleWhileDown);
          sub_LowerArmSubsystem.m_contract();
          sub_UpperArmToSetpoint.SetArm(desired.getAngle());
        }
        else 
        {
          sub_UpperArmToSetpoint.SetArm(desired.getAngle());
          sub_LowerArmSubsystem.m_contract();
        }
      }
      else
      {
        if(desired.getAngle() < ArmConstants.MinAngleWhileDown)
        {
          sub_UpperArmToSetpoint.SetArm(ArmConstants.MinAngleWhileDown);
          sub_LowerArmSubsystem.m_extend();
          sub_UpperArmToSetpoint.SetArm(desired.getAngle());
        }
        else
        {
          sub_UpperArmToSetpoint.SetArm(desired.getAngle());
          sub_LowerArmSubsystem.m_extend();
        } 
      }
    }
  }
}
