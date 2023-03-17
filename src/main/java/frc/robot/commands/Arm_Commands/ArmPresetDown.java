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

public class ArmPresetDown extends CommandBase {
  /** Creates a new ArmPresets. */
  UpperArmToSetpoint sub_UpperArmToSetpoint;
  LowerArmSubsystem sub_LowerArmSubsystem;

  public ArmPresetDown(UpperArmToSetpoint UpperArmToSetpoint, LowerArmSubsystem sub_LowerArmSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    sub_UpperArmToSetpoint = UpperArmToSetpoint;
    this.sub_LowerArmSubsystem = sub_LowerArmSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    int current = currentPreset();
    int newIndex = (current == -1) ? ArmConstants.presets.length-1 : Math.max(current-1, 0);
    ArmPreset desired = ArmConstants.presets[newIndex];
    sub_UpperArmToSetpoint.MoveArmPath(desired, sub_LowerArmSubsystem);
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
    return true;
  }
}
