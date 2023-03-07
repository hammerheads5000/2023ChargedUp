// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.LowerArmSubsystem;
import frc.robot.subsystems.UpperArmManual;

public class MoveArmManualCommand extends CommandBase {
  /** Creates a new MoveArmManualCommand. */

  private final Joystick arm = new Joystick(1);
  public final JoystickButton UpperArmIncreaseButton = new JoystickButton(arm, XboxController.Button.kLeftBumper.value); //works
  public final JoystickButton UpperArmDecreaseButton = new JoystickButton(arm, XboxController.Button.kRightBumper.value); //works
  private final JoystickButton lowerArmIncreaseButton = new JoystickButton(arm, XboxController.Button.kBack.value); //works
  private final JoystickButton lowerArmDecreaseButton = new JoystickButton(arm, XboxController.Button.kStart.value); //works
  private final UpperArmManual m_UpperArmManual;
  private final LowerArmSubsystem m_LowerArmSubsystem;
 
  public MoveArmManualCommand(UpperArmManual upperArmManual, LowerArmSubsystem lowerArmSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
 
    m_UpperArmManual = upperArmManual;
    m_LowerArmSubsystem = lowerArmSubsystem;
    addRequirements(upperArmManual);
    addRequirements(upperArmManual);
    addRequirements(lowerArmSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_LowerArmSubsystem.m_initializeSolenoid();
    m_LowerArmSubsystem.m_enableCompressor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    //Upper Arm Move Down
    if(UpperArmDecreaseButton.getAsBoolean())
    {
      m_UpperArmManual.moveDown(.3);
    }

    //Upper Arm Move Up
    else if(UpperArmIncreaseButton.getAsBoolean())
    {
      m_UpperArmManual.moveUp(.3);
    }

    //Lower Arm Move Down
    if(lowerArmDecreaseButton.getAsBoolean())
    {
      m_LowerArmSubsystem.m_contract();
    }

    //Lower Arm Move Up
    else if(lowerArmIncreaseButton.getAsBoolean())
    {
      m_LowerArmSubsystem.m_extend();
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
