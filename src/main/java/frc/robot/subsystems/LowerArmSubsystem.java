// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.playingwithfusion.CANVenom.BrakeCoastMode;

public class LowerArmSubsystem extends SubsystemBase {
  private boolean isUp = true;

  private static Compressor phCompressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
  
  private static DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);

  /** Creates a new LowerArmSubsystem. */
  public LowerArmSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void m_enableCompressor() {
    phCompressor.enableDigital();
  }

  public void m_initializeSolenoid() {
    doubleSolenoid.set(kForward);
  }

  public void m_disableCompressor() {
    phCompressor.disable();
  }

  public void m_toggle() {
    doubleSolenoid.toggle();
  }

  public void m_extend() {
    doubleSolenoid.set(kForward);
  }

  public void m_contract() {
    doubleSolenoid.set(kReverse);
  }

  public double currentPressure() {
    return phCompressor.getPressure();
  }

  public boolean checkState()
  {
    return isUp;
  }

  public void setIsUp(boolean set)
  {
    isUp = set;
  }
}
