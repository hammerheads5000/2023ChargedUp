// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UpperArmManual extends SubsystemBase {

  /*Behold, My variables */
    TalonFX ArmMotor = new TalonFX(3, "Bobby");
    

    /* Creates a new Pneumatics subsystem */
    public UpperArmManual() 
    {
      ArmMotor.setNeutralMode(NeutralMode.Brake);
    }

    //Moves arm up at a given speed
    public void moveUp(double speed)
    {
      ArmMotor.set(TalonFXControlMode.PercentOutput, speed); 
    }

    //Moves arm down at a given speed
    public void moveDown(double speed)
    {
        ArmMotor.set(TalonFXControlMode.PercentOutput, -speed);
    }

    public void Move(double speed)
    {
      ArmMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
    
    //keeps the arm from never decelerating
    public void stop()
    {
      ArmMotor.set(TalonFXControlMode.Disabled,0);
    }
}
