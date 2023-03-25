// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

public class AutoBalanceCommand extends CommandBase {
  /** Creates a new AutoBalanceCommand. */
  Swerve s_swerve;
  WPI_Pigeon2 pigeon;
  boolean balanced = false;
  Timer timer = new Timer();

  public AutoBalanceCommand(Swerve s_swerve, WPI_Pigeon2 pigeon) {
    this.s_swerve = s_swerve;
    this.pigeon = pigeon;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitch = pigeon.getPitch();
    SmartDashboard.putNumber("pitch", pitch);
    if (Math.abs(pitch) > AutoConstants.balanceZeroTolerance){
      double speed = -pitch*AutoConstants.balanceSensitivity; // speed proportional to pitch
      speed = MathUtil.clamp(speed, -AutoConstants.balanceSpeed, AutoConstants.balanceSpeed); // clamp to max speed
      s_swerve.drive(speed, 0, 0, true);
    }
    else {
      s_swerve.drive(0, 0, 0, true);
      balanced = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_swerve.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return balanced || timer.advanceIfElapsed(AutoConstants.maxBalanceTime);
  }
}
