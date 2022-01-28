package com.swervedrivespecialties.exampleswerve.autonomous;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.IntakeIndexSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.ShooterSubsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class GalacticSearch {
    
    private DrivetrainSubsystem drivetrain;
    private ShooterSubsystem shooter;
    private IntakeIndexSubsystem intakeIndex;
    private double tempAutoTicks;
    private int autoStep;
    private double turn;
    public AutoUtilities util;
    private double F;
    private double S;
    private double ticks;
    private NetworkTable lime;
    private boolean ballCheckIsFinished = false;
    public double[][] selectedPath;
    
    //path[] = {forwardSpeed, strafeSpeed, angle, distance}
    public double[][] redA = {
        {-0.5, 0.09, 0, 62}, //1
        {-0.5, 0.25, 0, 67}, //2
        {0, -0.5, 0, 90}, //3
        {-0.5, 0, 0, 160} //4        
    };
    public double[][] redB = {
        {-0.5, -0.15, 0, 62}, //1
        {0, 0.5, 0, 66}, //2
        {-0.5, 0, 0, 60}, //3
        {0, -0.5, 0, 54}, //4
        {-0.5, 0, 0, 60}, //5
        {-0.5, 0, 0, 126} //6   
    };

    public double[][] blueA = {
        {-0.5, 0.24, 0, 166}, //1
        {-0.1, -0.5, 0, 92}, //2
        {-0.5, 0, 0, 12}, //3
        {-0.5, 0.25, 0, 67},
        {-0.5, 0, 0, 66} //5   
    };
    public double[][] blueB = {
        {-0.5, 0.12, 0, 154}, //1
        {0, -0.5, 0, 60}, //2
        {-0.5, 0, 0, 60}, //3
        {0, 0.5, 0, 60}, //4
        {-0.5, 0, 0, 60}, //5
        {-0.5, 0, 0, 24} //6   
    };

    public GalacticSearch(DrivetrainSubsystem d, ShooterSubsystem s, IntakeIndexSubsystem i){
        drivetrain = d;
        intakeIndex = i;
        shooter = s;

        tempAutoTicks = drivetrain.getFRDEnc().getPosition();

        util = new AutoUtilities();

        lime = NetworkTableInstance.getDefault().getTable("limelight");

        drivetrain.resetGyroscope();
        autoStep = 0;
        turn = 0;
    }

    public void checkBalls(){
        double targetX;

        if(!ballCheckIsFinished){
            if(lime.getEntry("tv").getDouble(0) == 1){
                targetX = lime.getEntry("tx").getDouble(0);
                if(targetX > -2 && targetX < 2){
                    selectedPath = redA;
                    ballCheckIsFinished = true;
                }else if(targetX < -21){
                    selectedPath = redB;
                    ballCheckIsFinished = true;
                }else if(targetX > 20 && targetX < 24){
                    selectedPath = blueA;
                    ballCheckIsFinished = true;
                }else if(targetX > 9 && targetX < 14){
                    selectedPath = blueB;
                    ballCheckIsFinished = true;
                }
            }
        }   
    }
    

    public boolean ballCheckFinished(){
        return ballCheckIsFinished;
    }


    public void runAuto(){
        if(autoStep < selectedPath.length){
            runStep(selectedPath[autoStep]);
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
