package com.swervedrivespecialties.exampleswerve.autonomous;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.IntakeIndexSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.ShooterSubsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class BasicAuto {
    
    private DrivetrainSubsystem drivetrain;
    private ShooterSubsystem shooter;
    private IntakeIndexSubsystem intakeIndex;
    private double tempAutoTicks;
    private int autoStep = 0;
    public AutoUtilities util;
    private NetworkTable lime;

    public BasicAuto(DrivetrainSubsystem d, ShooterSubsystem s, IntakeIndexSubsystem i){
        drivetrain = d;
        intakeIndex = i;
        shooter = s;

        tempAutoTicks = drivetrain.getFRDEnc().getPosition();

        lime = NetworkTableInstance.getDefault().getTable("limelight");

        util = new AutoUtilities();

        drivetrain.resetGyroscope();
        autoStep = 0;
    }

    //path[] = {forwardSpeed, strafeSpeed, angle, distance}
    public double[][] path = {
        {-0.5, 0, 0, 25},
        {0, 0, 0, 0} //1
    };

    public void runAuto(){
        if(autoStep < path.length){
            runStep(path[autoStep]);
            shooter.shoot(0);
        }else{
            shooter.shoot(-(util.getShootRPM(util.getZone()) + 200));
            drivetrain.drive(new Translation2d(0,0), target(), true);
            intakeIndex.FireBalls(util.getShootRPM(util.getZone()), shooter.getLeftVelocity());
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

    public double target(){
        double rotation = 0;
        double offset = 3.5;

        if(lime.getEntry("tv").getDouble(0) == 1){
            if(Math.abs(lime.getEntry("tx").getDouble(0)) > 0.3/* && lime.getEntry("getpipe").getDouble(0) == 0*/){
                
                rotation = lime.getEntry("tx").getDouble(0)-offset;
                rotation = rotation*0.01667;
            }else{
             rotation = 0;
            }
        }

        return rotation;
    }

}
