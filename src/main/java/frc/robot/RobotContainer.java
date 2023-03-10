// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.Arm_Commands.ArmPresets;
import frc.robot.commands.Arm_Commands.ManualLowerArmDownCommand;
import frc.robot.commands.Arm_Commands.ManualLowerArmUpCommand;
import frc.robot.commands.Arm_Commands.ManualUpperArmDecreaseCommand;
import frc.robot.commands.Arm_Commands.ManualUpperArmIncreaseCommand;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick arm = new Joystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroWheels = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value); //Basically useless but probably works

  /* Arm Buttons */
  private final JoystickButton clawRotation = new JoystickButton(arm, XboxController.Button.kY.value);
  public final JoystickButton ArmSetButton = new JoystickButton(arm, XboxController.Button.kX.value); //works (probably)
  public final JoystickButton clawButton = new JoystickButton(arm, XboxController.Button.kA.value); //works
  public final JoystickButton UpperArmIncreaseButton = new JoystickButton(arm, XboxController.Button.kLeftBumper.value); //works
  public final JoystickButton UpperArmDecreaseButton = new JoystickButton(arm, XboxController.Button.kRightBumper.value); //works
  private final JoystickButton lowerArmIncreaseButton = new JoystickButton(arm, XboxController.Button.kBack.value); //works
  private final JoystickButton lowerArmDecreaseButton = new JoystickButton(arm, XboxController.Button.kStart.value); //works
  
  // limit switches 
  DigitalInput Lower_ArmBackwardsSwitch = new DigitalInput(2);
  DigitalInput Lower_ArmForwardsSwitch  = new DigitalInput(3);
  DigitalInput Upper_MaxWhileForwardsSwitch = new DigitalInput(4); 
  DigitalInput Upper_MaxWhileBackwardsSwitch = new DigitalInput(0);
  DigitalInput Upper_BringArmUpSafetySwitch = new DigitalInput(5);
  DigitalInput Upper_AtStowSwitch = new DigitalInput(1);

  //Motors 
  TalonFX UpperMotor = new TalonFX(3, "Bobby");
  TalonFX LowerMotor = new TalonFX(26, "Bobby");
  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();
  private final UpperArmManual sub_UpperArmManual = new UpperArmManual();
  private final LowerArmSubsystem sub_LowerArmSubsystem = new LowerArmSubsystem();
  private final ClawSubsystem sub_ClawSubsystem = new ClawSubsystem();
  private final UpperArmToSetpoint sub_ArmToSetpoint = new UpperArmToSetpoint();
  /* Commands */
  private final ClawCommand cmd_ClawCommand = new ClawCommand(sub_ClawSubsystem);
  private final ManualLowerArmDownCommand cmd_ManualLowerArmDownCommand = new ManualLowerArmDownCommand(sub_LowerArmSubsystem);
  private final ManualLowerArmUpCommand cmd_ManualLowerArmUpCommand = new ManualLowerArmUpCommand(sub_LowerArmSubsystem);
  private final ManualUpperArmDecreaseCommand cmd_ManualUpperArmDecreaseCommand = new ManualUpperArmDecreaseCommand(sub_UpperArmManual);
  private final ManualUpperArmIncreaseCommand cmd_UpperArmIncreaseCommand = new ManualUpperArmIncreaseCommand(sub_UpperArmManual);
  private final ArmPresets cmd_ArmPresets = new ArmPresets(sub_ArmToSetpoint);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    zeroWheels.onTrue(new InstantCommand(() -> s_Swerve.zeroWheels()));
    
    UpperArmDecreaseButton.onTrue(new InstantCommand(() -> sub_UpperArmManual.moveUp(0.3)));
    UpperArmIncreaseButton.onTrue(new InstantCommand(() -> sub_UpperArmManual.moveDown(0.3)));
    lowerArmDecreaseButton.onTrue(cmd_ManualLowerArmDownCommand);
    lowerArmIncreaseButton.onTrue(cmd_ManualLowerArmUpCommand);
    ArmSetButton.onTrue(cmd_ArmPresets);
    UpperArmDecreaseButton.onFalse(new InstantCommand(() -> sub_UpperArmManual.stop()));
    UpperArmIncreaseButton.onFalse(new InstantCommand(() -> sub_UpperArmManual.stop()));
    
    
    clawButton.onTrue(cmd_ClawCommand);
    clawRotation.onTrue(new InstantCommand(() -> sub_ClawSubsystem.rotate()));
  }
}
