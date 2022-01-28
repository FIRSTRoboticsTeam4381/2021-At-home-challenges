package com.swervedrivespecialties.exampleswerve.autonomous;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.IntakeIndexSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.ShooterSubsystem;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import com.swervedrivespecialties.exampleswerve.autonomous.AutoUtilities;

public class Test {
    
    private DrivetrainSubsystem drivetrain;
    private ShooterSubsystem shooter;
    private IntakeIndexSubsystem intakeIndex;
    private double tempAutoTicks;
    private int autoStep;
    private double turn;
    public AutoUtilities util;
    

    public Test(DrivetrainSubsystem d, ShooterSubsystem s, IntakeIndexSubsystem i){
        drivetrain = d;
        intakeIndex = i;
        shooter = s;

        tempAutoTicks = drivetrain.getFRDEnc().getPosition();

        util = new AutoUtilities();

        drivetrain.resetGyroscope();
        autoStep = 0;
        turn = 0;
    }

    public void runAuto(){
    
        switch(autoStep){
            case 0:
                if(Math.abs(tempAutoTicks - drivetrain.getFRDEnc().getPosition()) < util.inchesToTicks(58)){
                    drivetrain.drive(new Translation2d(0, 0.25), 0, true);
                }else{
                    drivetrain.drive(new Translation2d(0,0), 0, true);
                    tempAutoTicks = drivetrain.getFRDEnc().getPosition();
                    autoStep = 1;
                }
                break;
            case 1:
                if(Math.abs(tempAutoTicks - drivetrain.getFRDEnc().getPosition()) < util.inchesToTicks(116)){
                    drivetrain.drive(new Translation2d(-0.25, 0), 0, true);
                }else{
                    drivetrain.drive(new Translation2d(0,0), 0, true);
                    tempAutoTicks = drivetrain.getFRDEnc().getPosition();
                    autoStep = 2;
                }
                break;
            case 2:
                if(Math.abs(tempAutoTicks - drivetrain.getFRDEnc().getPosition()) < util.inchesToTicks(58)){
                    drivetrain.drive(new Translation2d(0, -0.25), 0, true);
                    shooter.setAutoShoot(1);
                }else{
                    drivetrain.drive(new Translation2d(0,0), 0, true);
                    tempAutoTicks = drivetrain.getFRDEnc().getPosition();
                    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
                    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
                    autoStep = 3;
                }
                break;
            case 3:
                if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1){
                    if(Math.abs(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0)) > 0.7){
                        turn = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
                        turn = turn*0.01667;
                        
                        drivetrain.drive(new Translation2d(0,0), turn, true);
                    }else{
                        autoStep = 4;
                    }
                }
                break;
            case 4:
                //intakeIndex.setAutoFire(1);
                break;
        }
    } 

}
