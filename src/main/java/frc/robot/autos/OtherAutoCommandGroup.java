// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.ArmSet;
import frc.robot.commands.AutoArmSet;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.LowerArmCommand;
import frc.robot.subsystems.ArmToSetpoint;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LowerArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OtherAutoCommandGroup extends SequentialCommandGroup {
  /** Creates a new autoCommandGroup. */
  public OtherAutoCommandGroup(Swerve s_AutoSwerve, LowerArmSubsystem sub_LowerArmSubsystem, ArmToSetpoint sub_ArmToSetpoint, int angle, ClawSubsystem sub_ClawSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(
    new AutoArmSet(sub_ArmToSetpoint, angle),
    new LowerArmCommand(sub_LowerArmSubsystem),
    new ClawCommand(sub_ClawSubsystem),
    new manualAuto(s_AutoSwerve, "2mPath.wpilib.json"),
    //new manualAuto(s_AutoSwerve, "part2Path.wpilib.json")
    new AutoArmSet(sub_ArmToSetpoint, angle),
    new LowerArmCommand(sub_LowerArmSubsystem),
    new ClawCommand(sub_ClawSubsystem)


    );
  }
}