// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm_Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.UpperArmSubsystem;

public class ManualUpperArmDecreaseCommand extends CommandBase {

  private final UpperArmSubsystem m_UpperArmManual;
  Joystick controller = new Joystick(1);
  JoystickButton decrease = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
  public ManualUpperArmDecreaseCommand(UpperArmSubsystem m_UpperArmManual) 
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
    m_UpperArmManual.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
