// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ArmToSetpoint;

public class MoveArmManualCommand extends CommandBase {
  /** Creates a new MoveArmManualCommand. */

  private final Joystick arm = new Joystick(1);
  public final JoystickButton UpperArmIncreaseButton = new JoystickButton(arm, XboxController.Button.kLeftBumper.value); //works
  public final JoystickButton UpperArmDecreaseButton = new JoystickButton(arm, XboxController.Button.kRightBumper.value); //works
  private final JoystickButton lowerArmIncreaseButton = new JoystickButton(arm, XboxController.Button.kBack.value); //works
  private final JoystickButton lowerArmDecreaseButton = new JoystickButton(arm, XboxController.Button.kStart.value); //works
  private final ArmToSetpoint m_lowerArmToSetpoint;
  private final ArmToSetpoint m_upperArmToSetpoint;
  public MoveArmManualCommand(ArmToSetpoint UpperArmToSetpoint, ArmToSetpoint LowerArmToSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lowerArmToSetpoint = LowerArmToSetpoint;
    m_upperArmToSetpoint = UpperArmToSetpoint;
    addRequirements(UpperArmToSetpoint);
    addRequirements(LowerArmToSetpoint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    int lowerTemp_EncoderCheck = m_lowerArmToSetpoint.EncoderCheck();
    int upperTemp_EncoderCheck = m_upperArmToSetpoint.EncoderCheck();
    if(UpperArmDecreaseButton.getAsBoolean())
    {
      m_upperArmToSetpoint.moveDown(.3, upperTemp_EncoderCheck);
    }
    else if(UpperArmIncreaseButton.getAsBoolean())
    {
      m_upperArmToSetpoint.moveUp(.3,upperTemp_EncoderCheck);
    }
    else if(lowerArmDecreaseButton.getAsBoolean())
    {
      m_lowerArmToSetpoint.moveDown(.3,lowerTemp_EncoderCheck);
    }
    else if(lowerArmIncreaseButton.getAsBoolean())
    {
      m_lowerArmToSetpoint.moveUp(.3, lowerTemp_EncoderCheck);
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
