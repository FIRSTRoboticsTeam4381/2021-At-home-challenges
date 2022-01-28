package com.swervedrivespecialties.exampleswerve.autonomous;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.IntakeIndexSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.ShooterSubsystem;

public class TemplateAuto {
    
    private DrivetrainSubsystem drivetrain;
    private ShooterSubsystem shooter;
    private IntakeIndexSubsystem intakeIndex;
    private double tempAutoTicks;
    private int autoStep;
    private double turn;
    public AutoUtilities util;
    

    public TemplateAuto(DrivetrainSubsystem d, ShooterSubsystem s, IntakeIndexSubsystem i){
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

                break;
            case 1:

                break;
        }
    }

}
