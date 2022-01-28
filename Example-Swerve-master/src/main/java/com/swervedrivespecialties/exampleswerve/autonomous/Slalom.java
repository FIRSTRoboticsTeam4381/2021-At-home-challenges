package com.swervedrivespecialties.exampleswerve.autonomous;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.IntakeIndexSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class Slalom {
    
    private DrivetrainSubsystem drivetrain;
    private ShooterSubsystem shooter;
    private IntakeIndexSubsystem intakeIndex;
    private double tempAutoTicks;
    private int autoStep;
    private double turn;
    public AutoUtilities util;

    public Slalom(DrivetrainSubsystem d, ShooterSubsystem s, IntakeIndexSubsystem i){
        drivetrain = d;
        intakeIndex = i;
        shooter = s;

        tempAutoTicks = drivetrain.getFRDEnc().getPosition();

        util = new AutoUtilities();

        drivetrain.resetGyroscope();
        autoStep = 0;
        turn = 0;
    }

    public double[][] path = {
        {-0.5, 0, 0, 46},
        {0, -0.5, 0, 55},
        {-0.5, 0, 0, 185},
        {0, 0.5, 0, 65},
        {-0.5, 0, 0, 70},
        {0, -0.5, 0, 60},
        {0.5, 0, 0, 55},
        {0, 0.5, 0, 65},
        {0.5, 0, 0, 168},
        {0, -0.5, 0, 60},
        {0.5, 0, 0, 40}
    };

    public void runAuto(){
        if(autoStep < path.length){
            runStep(path[autoStep]);
        }
    }

    //values[] = {forwardSpeed, strafeSpeed, angle, distance}
    public void runStep(double[] values){
        if(Math.abs(tempAutoTicks - drivetrain.getFRDEnc().getPosition()) < util.inchesToTicks(values[3])){
            drivetrain.drive(new Translation2d(values[0],values[1]), util.getTurnCorrect(values[2], drivetrain.getGyroAngle()), true);
        }else{
            tempAutoTicks = drivetrain.getFRDEnc().getPosition();
            autoStep++;
        }
    }

}
