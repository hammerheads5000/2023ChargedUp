// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
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
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value); //Basically useless but probably works
  private final JoystickButton intakeRaiseButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value); //works
  private final JoystickButton intakeClawButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value); ///works
  private final JoystickButton intakeMotorButton = new JoystickButton(driver, XboxController.Button.kA.value); //works
  private final JoystickButton zeroWheels = new JoystickButton(driver, XboxController.Button.kB.value); //?

  /* Arm Buttons */
  private final JoystickButton clawRotation = new JoystickButton(arm, XboxController.Button.kY.value); //doesn't work
  private final JoystickButton lowerArmButton = new JoystickButton(arm, XboxController.Button.kB.value); //works
  public final JoystickButton ArmSetButton = new JoystickButton(arm, XboxController.Button.kX.value); //works (probably)
  public final JoystickButton clawButton = new JoystickButton(arm, XboxController.Button.kA.value); //works
  public final JoystickButton ArmIncreaseButton = new JoystickButton(arm, XboxController.Button.kLeftBumper.value); //works
  public final JoystickButton ArmDecreaseButton = new JoystickButton(arm, XboxController.Button.kRightBumper.value); //works
  private final JoystickButton lowerArmIncreaseButton = new JoystickButton(arm, XboxController.Button.kBack.value); //works
  private final JoystickButton lowerArmDecreaseButton = new JoystickButton(arm, XboxController.Button.kStart.value); //works
  
  // limit switches 
  
  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
 
  private final ArmToSetpoint sub_ArmToSetpoint = new ArmToSetpoint();
  private final LowerArmSubsystem sub_LowerArmSubsystem = new LowerArmSubsystem();
  private final ClawSubsystem sub_ClawSubsystem = new ClawSubsystem();

  private final IntakeSubsystem sub_IntakeSubsystem = new IntakeSubsystem();

  /* Commands */
  private final ArmSet cmd_ArmSet = new ArmSet(sub_ArmToSetpoint);
  private final LowerArmCommand cmd_LowerArmCommand = new LowerArmCommand(sub_LowerArmSubsystem);
  private final ClawCommand cmd_ClawCommand = new ClawCommand(sub_ClawSubsystem);
  private final IntakeLiftCommand cmd_IntakeLiftCommand = new IntakeLiftCommand(sub_IntakeSubsystem);
  private final IntakeClawCommand cmd_IntakeClawCommand = new IntakeClawCommand(sub_IntakeSubsystem);

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
    
    ArmSetButton.whileTrue(cmd_ArmSet);
    ArmDecreaseButton.onTrue(new InstantCommand(() -> sub_ArmToSetpoint.moveUp(0.3)));
    ArmIncreaseButton.onTrue(new InstantCommand(() -> sub_ArmToSetpoint.moveDown(0.3)));
    ArmDecreaseButton.onFalse(new InstantCommand(() -> sub_ArmToSetpoint.stop()));
    ArmIncreaseButton.onFalse(new InstantCommand(() -> sub_ArmToSetpoint.stop()));
    
    lowerArmButton.onTrue(cmd_LowerArmCommand);

    clawButton.onTrue(cmd_ClawCommand);
    clawRotation.whileTrue(new InstantCommand(() -> sub_ClawSubsystem.rotate(arm.getXChannel())));

    double lowerArmMotorPercentOutput = 0.15;
    lowerArmIncreaseButton.onTrue(new InstantCommand(() -> sub_LowerArmSubsystem.m_increaseMotor(lowerArmMotorPercentOutput)));
    lowerArmDecreaseButton.onTrue(new InstantCommand(() -> sub_LowerArmSubsystem.m_decreaseMotor(lowerArmMotorPercentOutput)));
    lowerArmIncreaseButton.onFalse(new InstantCommand(() -> sub_LowerArmSubsystem.m_stopMotor()));
    lowerArmDecreaseButton.onFalse(new InstantCommand(() -> sub_LowerArmSubsystem.m_stopMotor()));

    intakeRaiseButton.whileTrue(cmd_IntakeLiftCommand);
    intakeClawButton.onTrue(new InstantCommand(() -> sub_IntakeSubsystem.m_extendGrabber()));
    intakeClawButton.onFalse(new InstantCommand(() -> sub_IntakeSubsystem.m_contractGrabber()));
    intakeMotorButton.whileTrue(new InstantCommand(() -> sub_IntakeSubsystem.m_activateIntakeMotor(1)));
    intakeMotorButton.whileFalse(new InstantCommand(() -> sub_IntakeSubsystem.m_stopIntakeMotor()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new quincyCommandGroup(sub_IntakeSubsystem);
  }


}
