package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmToSetpoint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class ArmSet extends CommandBase 
{
    private final ArmToSetpoint m_ArmToSetpoint;


    public ArmSet(ArmToSetpoint subsystem)
    {
        m_ArmToSetpoint = subsystem;
        addRequirements(subsystem);
    }


    @Override
    public void initialize() 
    {
        SmartDashboard.putNumber("Desired Angle", 0);
    }


    @Override
    public void execute() 
    {
        m_ArmToSetpoint.MoveArm(SmartDashboard.getNumber("Desired Angle", 0));
    }


    @Override
    public void end(boolean interrupted) 
    {

    }


    @Override
    public boolean isFinished() {
    return false;
    }   
}