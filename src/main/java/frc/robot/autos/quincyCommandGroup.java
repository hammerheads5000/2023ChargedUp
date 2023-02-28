// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeClawCommand;
import frc.robot.commands.LowerArmCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LowerArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class quincyCommandGroup extends SequentialCommandGroup {
  /** Creates a new quincyCommandGroup. */
  public quincyCommandGroup(IntakeSubsystem sub_IntakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(
            new WaitCommand(20),
    new IntakeClawCommand(sub_IntakeSubsystem)


    );
  }
}
