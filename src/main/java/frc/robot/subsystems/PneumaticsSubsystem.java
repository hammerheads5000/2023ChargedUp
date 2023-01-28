// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PneumaticsSubsystem extends SubsystemBase {

  private static Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  
  private static DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);

  /* Creates a new Pneumatics subsystem */
  public PneumaticsSubsystem() {
  }

  public void m_enableCompressor() {
    phCompressor.enableDigital();
  }

  public void m_initializeSolenoid() {
    doubleSolenoid.set(kReverse);
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

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public double currentPressure() {
    return phCompressor.getPressure();
  }

  public boolean compressorEnabled() {
    return phCompressor.isEnabled();
  }
}
