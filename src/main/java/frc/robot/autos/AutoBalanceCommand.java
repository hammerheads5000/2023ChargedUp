// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

public class AutoBalanceCommand extends CommandBase {
  /** Creates a new AutoBalanceCommand. */
  Swerve s_swerve;
  WPI_Pigeon2 pigeon;

  public AutoBalanceCommand(Swerve s_swerve, WPI_Pigeon2 pigeon) {
    this.s_swerve = s_swerve;
    this.pigeon = pigeon;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yaw = pigeon.getYaw();
    if (Math.abs(yaw) > AutoConstants.balanceToleranceDegrees){
      double speed = -yaw*AutoConstants.balanceSensitivity;
      speed = Math.min(Math.abs(speed), AutoConstants.maxBalanceSpeed) * Math.signum(speed); // cap speed at max speed
      s_swerve.drive(speed, 0, 0, true);
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
