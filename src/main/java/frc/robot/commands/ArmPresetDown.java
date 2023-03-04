// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LowerArmSubsystem;
import frc.robot.subsystems.UpperArmToSetpoint;
import frc.robot.Constants.ArmPresets;
import frc.robot.Constants.ArmPresets.ArmPresetEnum;

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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    switch (currentPreset()) {
      case GROUND:
        sub_LowerArmSubsystem.setIsUp(true);
        sub_UpperArmToSetpoint.MoveArm(ArmPresets.RestingAngle);
        break;
      case MIDPLATFORM:
        sub_LowerArmSubsystem.setIsUp(false);
        sub_UpperArmToSetpoint.MoveArm(ArmPresets.GroundAngle);
        break;
      case PORTAL:
        sub_LowerArmSubsystem.setIsUp(false);
        sub_UpperArmToSetpoint.MoveArm(ArmPresets.UpperPlatformAngle);
        break;
      case RESTING:
        break;
      case UPPERPLATFORM:
        sub_LowerArmSubsystem.setIsUp(true);
        sub_UpperArmToSetpoint.MoveArm(ArmPresets.MidPlatformAngle);
        break;
      default:
        break;
    }
  }

  public void MoveDown() {
    
  }

  public void MoveUp() {
    switch (currentPreset()) {
      case GROUND:
        sub_LowerArmSubsystem.setIsUp(false);
        sub_UpperArmToSetpoint.MoveArm(ArmPresets.MidPlatformAngle);
        break;
      case MIDPLATFORM:
      sub_LowerArmSubsystem.setIsUp(false);
        sub_UpperArmToSetpoint.MoveArm(ArmPresets.UpperPlatformAngle);
        break;
      case PORTAL:
        break;
      case RESTING:
      sub_LowerArmSubsystem.setIsUp(false);
        sub_UpperArmToSetpoint.MoveArm(ArmPresets.GroundAngle);
        break;
      case UPPERPLATFORM:
      sub_LowerArmSubsystem.setIsUp(true);
        sub_UpperArmToSetpoint.MoveArm(ArmPresets.PortalAngle);
        break;
      default:
        break;
    }
  }

  private ArmPresetEnum currentPreset() {
    final double TOLERANCE = 2;
    if(Math.abs(sub_UpperArmToSetpoint.angle - ArmPresets.GroundAngle) <= TOLERANCE) {
      return ArmPresetEnum.GROUND;
    }
    if(Math.abs(sub_UpperArmToSetpoint.angle - ArmPresets.UpperPlatformAngle) <= TOLERANCE) {
      return ArmPresetEnum.UPPERPLATFORM;
    }
    if(Math.abs(sub_UpperArmToSetpoint.angle - ArmPresets.MidPlatformAngle) <= TOLERANCE) {
      return ArmPresetEnum.MIDPLATFORM;
    }
    if(Math.abs(sub_UpperArmToSetpoint.angle - ArmPresets.RestingAngle) <= TOLERANCE) {
      return ArmPresetEnum.RESTING;
    }
    if(Math.abs(sub_UpperArmToSetpoint.angle - ArmPresets.PortalAngle) <= TOLERANCE) {
      return ArmPresetEnum.PORTAL;
    }
    return null;
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
