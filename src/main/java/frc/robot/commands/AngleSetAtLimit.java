// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
<<<<<<< HEAD:src/main/java/frc/robot/commands/UICommand.java
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UISubsystem;
=======
>>>>>>> 2c991b3833bb2c7985d3b41e6c304b3b4a6d623c:src/main/java/frc/robot/commands/AngleSetAtLimit.java

public class AngleSetAtLimit extends CommandBase {
  /** Creates a new AngleSetAtLimit. */
  DigitalInput Stow;
  public AngleSetAtLimit(DigitalInput Upper_ArmAtStow) {
    // Use addRequirements() here to declare subsystem dependencies.
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
