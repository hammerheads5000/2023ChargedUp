// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.platform.can.AutocacheState;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.autos.BalanceAutoCommandGroup;
import frc.robot.autos.PathAuto;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Arm_Commands.ArmPresetCommand;
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
  private final CommandXboxController driver =new CommandXboxController(0);
  private final Joystick driveJoystick = new Joystick(0);
  private final CommandXboxController arm = new CommandXboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final Trigger zeroWheels = driver.b();
  private final Trigger zeroGyro = driver.y(); //Basically useless but probably works

  /* Arm Buttons */
  private final Trigger clawRotation = arm.y();
  public final Trigger ArmSetButton = arm.x(); //works (probably)
  public final Trigger clawButton = arm.a(); //works
  public final Trigger UpperArmIncreaseButton = arm.leftBumper(); //works
  public final Trigger UpperArmDecreaseButton = arm.rightBumper(); //works
  private final Trigger lowerArmIncreaseButton = arm.back(); //works
  private final Trigger lowerArmDecreaseButton = arm.start(); //works
  
  private final Trigger ArmStateUpButton = arm.rightTrigger();
  private final Trigger ArmStateDownButton = arm.leftTrigger();

  private final Trigger armTopButton = arm.povUp();
  private final Trigger armMidButton = arm.povDown();
  private final Trigger armStowButton = arm.povLeft();
  private final Trigger armPortalButton = arm.povRight();
  //Motors 
  TalonFX UpperMotor = new TalonFX(3, "Bobby");
  TalonFX LowerMotor = new TalonFX(26, "Bobby");
  /* Subsystems */
  public final Swerve s_Swerve = new Swerve(driveJoystick, translationAxis, strafeAxis, rotationAxis, true, true);
  private final UpperArmSubsystem sub_UpperArmSubsystem = new UpperArmSubsystem();
  private final LowerArmSubsystem sub_LowerArmSubsystem = new LowerArmSubsystem();
  public final ClawSubsystem sub_ClawSubsystem = new ClawSubsystem();
  public final UISubsystem sub_UISubsystem = new UISubsystem(sub_LowerArmSubsystem, sub_ClawSubsystem, sub_UpperArmSubsystem);
  /* Commands */
  private final ClawCommand cmd_ClawCommand = new ClawCommand(sub_ClawSubsystem);
  private final ManualLowerArmDownCommand cmd_ManualLowerArmDownCommand = new ManualLowerArmDownCommand(sub_LowerArmSubsystem);
  private final ManualLowerArmUpCommand cmd_ManualLowerArmUpCommand = new ManualLowerArmUpCommand(sub_LowerArmSubsystem);
  private final ManualUpperArmDecreaseCommand cmd_ManualUpperArmDecreaseCommand = new ManualUpperArmDecreaseCommand(sub_UpperArmSubsystem);
  private final AutoBalanceCommand cmd_AutoBalanceCommand = new AutoBalanceCommand(s_Swerve, s_Swerve.gyro);
  private final ArmPresetCommand cmd_GroundPresetCommand = new ArmPresetCommand(ArmConstants.ground);
  private final ArmPresetCommand cmd_MidPlatformPresetCommand = new ArmPresetCommand(ArmConstants.midPlatform);
  private final ArmPresetCommand cmd_PortalPresetCommand = new ArmPresetCommand(ArmConstants.portal);
  private final ArmPresetCommand cmd_StowPresetCommand = new ArmPresetCommand(ArmConstants.stow);
  private final ArmPresetCommand cmd_UpperPlatformPresetCommand = new ArmPresetCommand(ArmConstants.upperPlatform);
  public final Initialize init = new Initialize(cmd_GroundPresetCommand, sub_LowerArmSubsystem, sub_ClawSubsystem);

  private final BalanceAutoCommandGroup auto_balance = new BalanceAutoCommandGroup(s_Swerve, cmd_AutoBalanceCommand);
  private final PathAuto auto_fullAuto = new PathAuto(s_Swerve, cmd_UpperPlatformPresetCommand, cmd_StowPresetCommand, sub_LowerArmSubsystem, sub_ClawSubsystem, cmd_AutoBalanceCommand, "Full Auto");
  private final PathAuto auto_longAuto = new PathAuto(s_Swerve, cmd_UpperPlatformPresetCommand, cmd_StowPresetCommand, sub_LowerArmSubsystem, sub_ClawSubsystem, cmd_AutoBalanceCommand, "Long Auto");
  private final PathAuto auto_shortAuto = new PathAuto(s_Swerve, cmd_UpperPlatformPresetCommand, cmd_StowPresetCommand, sub_LowerArmSubsystem, sub_ClawSubsystem, cmd_AutoBalanceCommand, "Short Auto");
  private final PathAuto auto_testAutoCone = new PathAuto(s_Swerve, cmd_UpperPlatformPresetCommand, cmd_StowPresetCommand, sub_LowerArmSubsystem, sub_ClawSubsystem, cmd_AutoBalanceCommand, "Test cone");
  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final ManualUpperArmIncreaseCommand cmd_UpperArmIncreaseCommand = new ManualUpperArmIncreaseCommand(sub_UpperArmSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   // sub_LowerArmToSetpoint.setDefaultCommand(cmd_ArmAtLimit);
   // sub_UpperArmToSetpoint.setDefaultCommand(cmd_ArmAtSwitch);
    // Configure the button bindings
    configureButtonBindings();
    configureAutoOptions();
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
    
    UpperArmDecreaseButton.onTrue(new InstantCommand(() -> sub_UpperArmSubsystem.moveUp(0.3)));
    UpperArmIncreaseButton.onTrue(new InstantCommand(() -> sub_UpperArmSubsystem.moveDown(0.3)));
    lowerArmDecreaseButton.onTrue(cmd_ManualLowerArmDownCommand);
    lowerArmIncreaseButton.onTrue(cmd_ManualLowerArmUpCommand);
    UpperArmDecreaseButton.onFalse(new InstantCommand(() -> sub_UpperArmSubsystem.stop()));
    UpperArmIncreaseButton.onFalse(new InstantCommand(() -> sub_UpperArmSubsystem.stop()));

    armTopButton.onTrue(cmd_UpperPlatformPresetCommand); 
    armMidButton.onTrue(cmd_MidPlatformPresetCommand); 
    armStowButton.onTrue(cmd_StowPresetCommand); 
    armPortalButton.onTrue(cmd_PortalPresetCommand); 
    
    clawButton.onTrue(cmd_ClawCommand);
    clawRotation.onTrue(new InstantCommand(() -> sub_ClawSubsystem.rotate()));

  }

  private void configureAutoOptions() {
    autoChooser.setDefaultOption("Full Auto", auto_fullAuto);
    autoChooser.addOption("Long Auto", auto_longAuto);
    autoChooser.addOption("Short Auto", auto_shortAuto);
    autoChooser.addOption("Test cone drop", auto_testAutoCone);

    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }

  public void swerveInit(boolean fieldRelative, boolean openLoop) {
    //s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driveJoystick, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));
  }
}
