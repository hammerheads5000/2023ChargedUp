// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class PlacePiece extends CommandBase { 
  private Swerve s_swerve;

  /** Creates a new PlacePiece. */
  public PlacePiece(Swerve s_swerve) {
    this.s_swerve = s_swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!inPosition()) {
      s_swerve.drive(0, -Constants.AutoConstants.scanVel, 0, true);
    }
  }

  private boolean inPosition() {
    // return true if position relative to april tag is correct
    return false;
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
