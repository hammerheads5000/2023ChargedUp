// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MaxCommandGroup extends SequentialCommandGroup {
  /** Creates a new MaxCommandGroup. */
  public MaxCommandGroup(Swerve s_AutoSwerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(
      //    Trajectory exampleTrajectory = new Trajectory();

      //new InstantCommand(() -> s_AutoSwerve.resetOdometry(exampleTrajectory.getInitialPose())),

        new InstantCommand(() -> s_AutoSwerve.drive(1,0,0,false)),       // Need to stop robot at end, likely different param to set FieldRelative
        new WaitCommand(2.0),
        new InstantCommand(() -> s_AutoSwerve.drive(0,0,0,false))   
    );
  }
}
