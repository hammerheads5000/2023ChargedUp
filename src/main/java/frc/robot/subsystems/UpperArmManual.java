// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.playingwithfusion.CANVenom.BrakeCoastMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.RegularConstants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
public class UpperArmManual extends SubsystemBase {

  /*Behold, My variables */
    TalonFX ArmMotor = new TalonFX(3, "Bobby");
    

    /* Creates a new Pneumatics subsystem */
    public UpperArmManual() 
    {
      ArmMotor.setNeutralMode(NeutralMode.Brake);
  

       /*sets the preset angles depending on which arm instance it is
      if(isUpperArm)
      {
        HomeAngle = 20;
        UpperAngle = 20;
      }
      else
      {
        HomeAngle = 1;
        UpperAngle = 3;
      } */
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
    

    //keeps the arm from never decelerating
    public void stop()
    {
      ArmMotor.set(TalonFXControlMode.Disabled,0);
    }
}
