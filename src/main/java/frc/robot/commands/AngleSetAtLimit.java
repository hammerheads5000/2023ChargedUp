// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmPresets;
import frc.robot.subsystems.UpperArmToSetpoint;

public class AngleSetAtLimit extends CommandBase {
  /** Creates a new AngleSetAtLimit. */
  DigitalInput Stow;
  UpperArmToSetpoint sub_UpperArmToSetpoint;
  public AngleSetAtLimit(DigitalInput Upper_ArmAtStow, UpperArmToSetpoint sub_UpperArmToSetpoint) {
    Stow = Upper_ArmAtStow;
    this.sub_UpperArmToSetpoint = sub_UpperArmToSetpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(Stow.get())
    {
      sub_UpperArmToSetpoint.AngleSet(ArmPresets.RestingAngle); // set angle at limit switch NEED TO SET CORRECTLY
    }
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
