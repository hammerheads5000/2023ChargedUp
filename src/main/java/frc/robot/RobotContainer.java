// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.platform.can.AutocacheState;

import edu.wpi.first.cameraserver.CameraServer;
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

import frc.robot.autos.*;
import frc.robot.commands.*;
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
  private final JoystickButton intakeRaiseButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value); //?
  private final JoystickButton intakeClawButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value); //?
  private final JoystickButton intakeMotorButton = new JoystickButton(driver, XboxController.Button.kA.value); //?

  /* Arm Buttons */
  private final JoystickButton clawRotation = new JoystickButton(arm, XboxController.Button.kY.value);
  private final JoystickButton lowerArmButton = new JoystickButton(arm, XboxController.Button.kB.value); //works
  //public final JoystickButton ArmSetButton = new JoystickButton(arm, XboxController.Button.kX.value); //works (probably)
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
  private final IntakeSubsystem sub_IntakeSubsystem = new IntakeSubsystem();
  private final EncoderCheck sub_EncoderCheck = new EncoderCheck(Lower_ArmBackwardsSwitch, Lower_ArmForwardsSwitch, Upper_MaxWhileForwardsSwitch, Upper_MaxWhileBackwardsSwitch, Upper_BringArmUpSafetySwitch, Upper_AtStowSwitch);
  /* Commands */
  //private final ArmSet cmd_ArmSet = new ArmSet(sub_UpperArmToSetpoint, sub_LowerArmToSetpoint);
  private final LowerArmCommand cmd_LowerArmCommand = new LowerArmCommand(sub_LowerArmSubsystem);
  private final ClawCommand cmd_ClawCommand = new ClawCommand(sub_ClawSubsystem);
  private final IntakeLiftCommand cmd_IntakeLiftCommand = new IntakeLiftCommand(sub_IntakeSubsystem);
  private final IntakeClawCommand cmd_IntakeClawCommand = new IntakeClawCommand(sub_IntakeSubsystem);
  private final MoveArmManualCommand cmd_MoveArmManualCommand = new MoveArmManualCommand(sub_UpperArmManual,sub_LowerArmSubsystem,sub_EncoderCheck);
  private final AutoBalanceCommand cmd_AutoBalanceCommand = new AutoBalanceCommand(s_Swerve, s_Swerve.gyro);
  
  private final BalanceAutoCommandGroup auto_balance = new BalanceAutoCommandGroup(s_Swerve, cmd_AutoBalanceCommand);
  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  //private final ArmAtLimit cmd_ArmAtLimit = new ArmAtLimit(sub_UpperArmToSetpoint, sub_LowerArmToSetpoint, Upper_BringArmUpSafetySwitch, Upper_AtStowSwitch, Lower_ArmForwardsSwitch, Lower_ArmBackwardsSwitch)
  //private final ArmAtLimit cmd_ArmAtSwitch = new ArmAtLimit(sub_UpperArmToSetpoint,sub_LowerArmToSetpoint, UpperArmLowerSwitch, UpperArmUpperSwitch, LowerArmLowerSwitch, LowerArmUpperSwitch);
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
    
   // ArmSetButton.whileTrue(cmd_ArmSet);
    UpperArmDecreaseButton.onTrue(cmd_MoveArmManualCommand);
    UpperArmIncreaseButton.onTrue(cmd_MoveArmManualCommand);
    UpperArmDecreaseButton.onFalse(new InstantCommand(() -> sub_UpperArmManual.stop()));
    UpperArmIncreaseButton.onFalse(new InstantCommand(() -> sub_UpperArmManual.stop()));
    
    
    clawButton.onTrue(cmd_ClawCommand);
    clawRotation.onTrue(new InstantCommand(() -> sub_ClawSubsystem.rotate()));

    double lowerArmMotorPercentOutput = 0.15;
    lowerArmIncreaseButton.onTrue(cmd_MoveArmManualCommand);
    lowerArmDecreaseButton.onTrue(cmd_MoveArmManualCommand);
    lowerArmIncreaseButton.onFalse(new InstantCommand(() -> sub_LowerArmSubsystem.m_stopMotor()));
    lowerArmDecreaseButton.onFalse(new InstantCommand(() -> sub_LowerArmSubsystem.m_stopMotor()));

    intakeRaiseButton.whileTrue(cmd_IntakeLiftCommand);
    intakeClawButton.onTrue(new InstantCommand(() -> sub_IntakeSubsystem.m_extendGrabber()));
    intakeClawButton.onFalse(new InstantCommand(() -> sub_IntakeSubsystem.m_contractGrabber()));
    intakeMotorButton.whileTrue(new InstantCommand(() -> sub_IntakeSubsystem.m_activateIntakeMotor(1)));
    intakeMotorButton.whileFalse(new InstantCommand(() -> sub_IntakeSubsystem.m_stopIntakeMotor()));

  }

  private void configureAutoOptions() {
    autoChooser.setDefaultOption("Auto balance", auto_balance);
    
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
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));
  }


}
