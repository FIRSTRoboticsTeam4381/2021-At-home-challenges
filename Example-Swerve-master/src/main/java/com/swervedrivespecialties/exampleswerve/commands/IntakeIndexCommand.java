package com.swervedrivespecialties.exampleswerve.commands;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.autonomous.AutoUtilities;
import com.swervedrivespecialties.exampleswerve.subsystems.IntakeIndexSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.ShooterSubsystem;

public class IntakeIndexCommand extends Command{
    
    private AutoUtilities util;

    public IntakeIndexCommand(){
        requires(IntakeIndexSubsystem.getInstance());

        util = new AutoUtilities();
    }

    protected void execute(){
     
        if(Robot.getOi().getSpecialJoystick().getRawButton(6)){
            IntakeIndexSubsystem.getInstance().FireBalls(util.getShootRPM(util.getZone()), ShooterSubsystem.getInstance().getLeftVelocity());
            //IntakeIndexSubsystem.getInstance().FireBalls(util.getShootRPM(util.getZone())-400, Math.abs(ShooterSubsystem.getInstance().getLeftVelocity()));
            //IntakeIndexSubsystem.getInstance().FireBalls(1000, ShooterSubsystem.getInstance().getLeftVelocity());
        }else if(Robot.getOi().getSpecialJoystick().getRawButton(12)){
            IntakeIndexSubsystem.getInstance().rawIndex(-0.3);
            IntakeIndexSubsystem.getInstance().rawIntake(0.5);
        }else if(Robot.getOi().getSpecialJoystick().getRawButton(2) || Robot.getOi().getPrimaryJoystick().getRawButton(Robot.getOi().getControlNum('p'))){
            IntakeIndexSubsystem.getInstance().Index();
        }else if(Robot.getOi().getSpecialJoystick().getRawButton(11) || IntakeIndexSubsystem.getInstance().autoStatus()){
            IntakeIndexSubsystem.getInstance().reverse();
        }else if(Robot.getOi().getSpecialJoystick().getRawButtonReleased(5) || 
                Robot.getOi().getSpecialJoystick().getRawButtonReleased(12) ||
                Robot.getOi().getSpecialJoystick().getRawButtonReleased(2) ||
                Robot.getOi().getPrimaryJoystick().getRawButtonReleased(Robot.getOi().getControlNum('p')) ||
                Robot.getOi().getSpecialJoystick().getRawButtonReleased(11))
                {
            IntakeIndexSubsystem.getInstance().rawIndex(0);
            IntakeIndexSubsystem.getInstance().rawIntake(0);
        }
    }

    @Override
    protected boolean isFinished(){
        return false;
    }
}
