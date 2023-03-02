// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;

public class ClawSubsystem extends SubsystemBase {
  public boolean isOpen = true;

  //private static Compressor phCompressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
  
  private static DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  private int currentAngle = 0;
  Servo rotationMotor = new Servo(4);
  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    rotationMotor.setAngle(currentAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void rotate()
  {
    //rotationMotor.set(direction);
    int angle;
    if (currentAngle == 0) {angle = 180;}
    else {angle = 0;}
    currentAngle = angle;
    rotationMotor.setAngle(angle);
  }

  public void m_enableCompressor() {
    //phCompressor.enableDigital();
  }

  public void m_initializeSolenoid() {
    doubleSolenoid.set(kReverse);
  }

  public void m_disableCompressor() {
    //phCompressor.disable();
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
    //return phCompressor.getPressure();
    return 0;
  }

  public void setState(boolean set)
  {
    isOpen = set;
  }

  public boolean getState()
  {
    return isOpen;
  }
}
