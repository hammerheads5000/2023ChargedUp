// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.NeutralOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class UpperArmSubsystem extends SubsystemBase {

  /*Behold, My variables */
    TalonFX armMotor = new TalonFX(3, "Bobby");
    DutyCycleEncoder armEncoder = new DutyCycleEncoder(9);

    /* Creates a new Pneumatics subsystem */
    public UpperArmSubsystem() 
    {
      MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
      motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
      armMotor.getConfigurator().apply(motorOutputConfigs);
    }

    //Moves arm up at a given speed
    public void moveUp(double speed)
    {
      armMotor.setControl(new DutyCycleOut(speed)); 
    }

    //Moves arm down at a given speed
    public void moveDown(double speed)
    {
      armMotor.setControl(new DutyCycleOut(-speed)); 
    }

    public void Move(double speed)
    {
      armMotor.setControl(new DutyCycleOut(speed)); 
    }

    public double getAngle()
    {
      return ((armEncoder.getAbsolutePosition() * 360) + ArmConstants.offset) % 360;
    }
    
    //keeps the arm from never decelerating
    public void stop()
    {
      armMotor.setControl(new NeutralOut());
    }
}
