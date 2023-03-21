// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmState;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.LowerArmSubsystem;
import frc.robot.subsystems.UpperArmSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmPresetCommand extends CommandBase {
  /** Creates a new ArmToPortalPresetCommand. */
  UpperArmSubsystem sub_UpperArmSubsystem;
  LowerArmSubsystem sub_LowerArmSubsystem;
  ArmState Preset;
  double LastError;
  ArmState LastDesiredArmState;
  boolean SamePIDInstance;
  Timer deltaTimer = new Timer();

  boolean finished = false;
  public ArmPresetCommand(ArmState Preset, UpperArmSubsystem sub_UpperArmSubsystem,LowerArmSubsystem sub_LowerArmSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Preset = Preset;
    deltaTimer.start();
    this.sub_LowerArmSubsystem = sub_LowerArmSubsystem;
    this.sub_UpperArmSubsystem = sub_UpperArmSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    deltaTimer.reset();
    SamePIDInstance = false;
    LastDesiredArmState = new ArmState(0, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ArmState current = new ArmState(sub_UpperArmSubsystem.getAngle(), sub_LowerArmSubsystem.checkState());
    ArmState desired = ArmPathfind(Preset, current);
    if(Preset.equals(current)) {
      finished = true;
      return;
    }
    double output = PID(desired,current, SamePIDInstance);
    sub_UpperArmSubsystem.Move(output);
    if(!current.getLowerArmUp() == Preset.getLowerArmUp()) {
      sub_LowerArmSubsystem.m_toggle();
    }
    SamePIDInstance = desired.equals(LastDesiredArmState);
   
    LastDesiredArmState = desired;
    SmartDashboard.putNumber("ouptut", output);
  }

  public double PID(ArmState desired,ArmState current, boolean sameInstance){
    double derivative;
    double error = desired.getAngle() - current.getAngle();
    double proportional = error * ArmConstants.kP;
    if(sameInstance) {
      derivative = ((error - LastError) / deltaTimer.get()) * ArmConstants.kD;
    }
    else {
      derivative = 0;
    }
    deltaTimer.reset();
    LastError = error;
    return derivative + proportional;
  }

  
  public ArmState ArmPathfind(ArmState desired, ArmState current)
  {
    if (desired.getLowerArmUp() == current.getLowerArmUp()) {
      return desired;
    }
    else if (desired.getLowerArmUp()) {
      if (current.getAngle() > ArmConstants.MaxAngleWhileUp) {
        return new ArmState(ArmConstants.MaxAngleWhileUp, current.getLowerArmUp());
      }
      else {
        return desired;
      }
    }
    else {
      if(current.getAngle() < ArmConstants.MinAngleWhileDown){
        return new ArmState(ArmConstants.MinAngleWhileDown, current.getLowerArmUp());
      }
      else {
        return desired;
      }
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sub_UpperArmSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
