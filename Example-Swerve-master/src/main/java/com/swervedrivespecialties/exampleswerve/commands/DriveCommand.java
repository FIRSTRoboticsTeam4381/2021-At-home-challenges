package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.autonomous.AutoUtilities;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.frcteam2910.common.robot.Utilities;


public class DriveCommand extends Command {

    public NetworkTable lime;
    public double offset;
    public AutoUtilities util;
    private boolean noTurn = false;

    public DriveCommand() {
        requires(DrivetrainSubsystem.getInstance());

        lime = NetworkTableInstance.getDefault().getTable("limelight");

        util = new AutoUtilities();
    }

    @Override
    protected void execute() {
        
        double throttle = 0.5;
        
        double x = lime.getEntry("tlong").getDouble(0);
        double distance = 0.0052*Math.pow(x,2) - 3.0557*x + 508.09;
        /*
        if(distance > 210){
            offset = 4.5;
        }else{
            offset = 5.5;
        }
        */
        offset = 3.5;


        /*
        if(Robot.getOi().getPrimaryJoystick().getRawAxis(3) > 0.8){
            if(Robot.getOi().getPrimaryJoystick().getRawAxis(2)>0.8){
                throttle = 0;
            }else{
                throttle = 1;
            }
        }else{
            throttle = 0.5;
        }
       */

        

        //if(Robot.getOi().getSpecialJoystick().getRawButtonPressed(10)){
           // noTurn = !noTurn;
        //}
        //SmartDashboard.putBoolean("Turn Lockout", noTurn);

       // if(Robot.getOi().getSpecialJoystick().getThrottle() < 1){
           //throttle = (Robot.getOi().getSpecialJoystick().getThrottle()+1)/2;
            throttle = 1;
           // }else{
            //throttle = ()+1.5)/2;
        
        //}

        /* DEMO CODE
        if(Robot.getOi().getPrimaryJoystick().getRawAxis(Robot.getOi().getControlNum('x')) > 0.9){
            throttle = 1;
        }else{
            throttle = 0.5;
        }*/

        double forward = Robot.getOi().getPrimaryJoystick().getRawAxis(Robot.getOi().getControlNum('f'))*throttle;
        forward = Utilities.deadband(forward);
        // Square the forward stick
        forward = Math.copySign(Math.pow(forward, 2.0), forward);

        double strafe = Robot.getOi().getPrimaryJoystick().getRawAxis(Robot.getOi().getControlNum('s'))*throttle;
        strafe = Utilities.deadband(strafe);
        // Square the strafe stick
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);

        double rotation;
        if(Robot.getOi().getSpecialJoystick().getRawButton(6) && (lime.getEntry("tv").getDouble(0) == 1)){
            if(Math.abs(lime.getEntry("tx").getDouble(0)) > 0.3 && lime.getEntry("getpipe").getDouble(0) == 0){
                
                rotation = lime.getEntry("tx").getDouble(0)-offset;
                rotation = rotation*0.01667;
            }else{
             rotation = 0;
            }
            SmartDashboard.putBoolean("LIMELIGHTING IT UP", true);
        }else{
            rotation = Robot.getOi().getPrimaryJoystick().getRawAxis(Robot.getOi().getControlNum('t'))*throttle;
            rotation = Utilities.deadband(rotation);
            // Square the rotation stick
            rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);
            SmartDashboard.putBoolean("LIMELIGHTING IT UP", false);
        }
        
        DrivetrainSubsystem.getInstance().drive(new Translation2d(forward, strafe), rotation, true);
        SmartDashboard.putNumber("Rotation", rotation);
    }
            /*else if(noTurn){
            rotation = util.getTurnCorrect(0, DrivetrainSubsystem.getInstance().getGyroAngle());
            SmartDashboard.putBoolean("LIMELIGHTING IT UP", false);*/
    @Override
    protected boolean isFinished() {
        return false;
    }
}

