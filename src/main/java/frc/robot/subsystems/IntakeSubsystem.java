// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class IntakeSubsystem extends SubsystemBase {

  //private static Compressor phCompressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
  
  private static DoubleSolenoid lift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);


  private final CANSparkMax intakeMotor = new CANSparkMax(15, MotorType.kBrushless);

  public boolean isGrabberOpen = true;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void m_enableCompressor() {
    //phCompressor.enableDigital();
  }

  public void m_initializeSolenoid() {
    lift.set(kReverse);
  
  }

  public void m_disableCompressor() {
    //phCompressor.disable();
  }

  public void m_toggle() {
    lift.toggle();
  }

  public void m_extendLift() {
    lift.set(kForward);
  }

  public void m_contractLift() {
    lift.set(kReverse);
  }

  public void m_extendGrabber(){

  }

  public void m_contractGrabber(){
  
  }

  public void m_activateIntakeMotor(double direction) {
    intakeMotor.set(direction);
  }

  public void m_stopIntakeMotor()
  {
    intakeMotor.set(0);
  }

  public double currentPressure() {
    //return phCompressor.getPressure();
    return 0;
  }

  public boolean getGrabberState() {
    return isGrabberOpen;
  }

  public void setGrabberState(boolean set) {
    isGrabberOpen = set;
  }
}
