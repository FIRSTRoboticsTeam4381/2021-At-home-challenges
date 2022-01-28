package com.swervedrivespecialties.exampleswerve.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.autonomous.AutoUtilities;
import com.swervedrivespecialties.exampleswerve.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command{

    private int zone = 0;
    public NetworkTable lime;
    private AutoUtilities util;

    public ShooterCommand(){
        requires(ShooterSubsystem.getInstance());

        util = new AutoUtilities();

    }

    protected void execute(){
        //At row 4 speed is 1891

        double shootSpeed;
        double rawThrottle;
        double shootRPM = 0;

        rawThrottle = Robot.getOi().getSpecialJoystick().getThrottle();
        shootSpeed = (-rawThrottle+1)/2;

    

        if(Robot.getOi().getSpecialJoystick().getRawButton(6)){
            ShooterSubsystem.getInstance().shoot(-(util.getShootRPM(util.getZone())+200));
            //ShooterSubsystem.getInstance().shoot(-util.getShootRPM(util.getZone()));
            //ShooterSubsystem.getInstance().shoot(-1891);
            //ShooterSubsystem.getInstance().shoot(-shootSpeed);
        }else if(ShooterSubsystem.getInstance().getAutoShoot()){
            ShooterSubsystem.getInstance().shoot(-1000);
        }else{
            //Idle for Accuracy Challenge
            //ShooterSubsystem.getInstance().shoot(-1000);

            //Idle for Power Port Challenge
            //ShooterSubsystem.getInstance().shoot(-1400);

            //Idle for Hyperdrive Challenge
            ShooterSubsystem.getInstance().shoot(0);
        }

        

    }

    @Override
    protected boolean isFinished(){
        return false;
    }
}
