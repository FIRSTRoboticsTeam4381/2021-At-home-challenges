package com.swervedrivespecialties.exampleswerve.autonomous;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.IntakeIndexSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class Bounce {
    
    private DrivetrainSubsystem drivetrain;
    private ShooterSubsystem shooter;
    private IntakeIndexSubsystem intakeIndex;
    private double tempAutoTicks;
    private int autoStep = 0;
    public AutoUtilities util;

    public Bounce(DrivetrainSubsystem d, ShooterSubsystem s, IntakeIndexSubsystem i){
        drivetrain = d;
        intakeIndex = i;
        shooter = s;

        tempAutoTicks = drivetrain.getFRDEnc().getPosition();

        util = new AutoUtilities();

        drivetrain.resetGyroscope();
        autoStep = 0;
    }

    //path[] = {forwardSpeed, strafeSpeed, angle, distance}
    public double[][] path = {
        {-0.5, 0, 0, 50}, //1
        {0, -0.5, 0, 50}, //2
        {0, 0.5, 0, 58}, //3
        {-0.5, 0, 0, 32}, //4
        {0, 0.5, 0, 60}, //5
        {-0.5, 0, 0, 55.7}, //6
        {0, -0.5, 0, 100}, //7
        {0, 0.5, 0, 125}, //8
        {-0.5, 0, 0, 92}, //9
        {0, -0.5, 0, 110}, //10
        {0, 0.5, 0, 65}, //11
        {-0.5, 0, 0, 40} //12
        
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
