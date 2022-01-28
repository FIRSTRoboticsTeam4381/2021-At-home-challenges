package com.swervedrivespecialties.exampleswerve.autonomous;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoUtilities {

    public NetworkTable lime;
    private boolean inAuto = false;

    public AutoUtilities(){
        lime = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double inchesToTicks(double inches){
        double ticks;

        ticks = (1.0485*inches);

        return ticks;
    }

    public int getZone(){
        int zone = 0;

        double x = lime.getEntry("tlong").getDouble(0);
        double distance = 0.0052*Math.pow(x,2) - 3.0557*x + 508.09;
        SmartDashboard.putNumber("Predicted In", distance);
        
        zone = 0;

        if(distance > 30 && distance < 80){
            zone = 1;
        }else if(distance > 80 && distance < 150){
            zone = 2;
        }else if(distance > 150 && distance < 210){
            zone = 3;
        }else if(distance > 210 && distance < 270){
            zone = 4;
        }

        return zone;
    }

    public double getShootRPM(int zone){
        double shootRPM = 0;
        switch(zone){
            case 1:
                shootRPM = 2600;
                break;
            case 2:
                shootRPM = 1891;
                //shootRPM = 1760;
                break;
            case 3:
                shootRPM = 2100;
                //shootRPM = 1960;
                break;
            case 4:
                shootRPM = 2325;
                break;
            default:
                shootRPM = 1891;
                break;
        }
        return shootRPM;
    }

    public double getTurnCorrect(double desiredAngle, double gyroAngle){
        double correct = 0;
        if(desiredAngle == 0 || desiredAngle == 360){
            if(gyroAngle < 180){
                correct = (-3.6/180)*Math.abs(gyroAngle-180)+3.6;
            }else if(gyroAngle > 180){
                correct = (3.6/180)*Math.abs(gyroAngle-180)-3.6;
            }
        }else{
            correct = (3.6/180)*(gyroAngle-desiredAngle);
        }
        
        return correct*0.2;
    }

    public double[] getArc(double enc, double radius, double maxPow){
        double[] output = {0,0};
        double angle;
        double F, S;

        angle = enc/radius;

        //maxpower affects radius

        F = Math.cos(angle)*radius;
        S = Math.sin(angle)*radius;

        F = F/(Math.sqrt(Math.pow(F, 2) + Math.pow(S, 2)));
        S = S/(Math.sqrt(Math.pow(F, 2) + Math.pow(S, 2)));

        output[0] = F*maxPow;
        output[1] = S*maxPow;

        return output;
    }


    public boolean drivePath(int currentStep, DrivetrainSubsystem drivetrain, int[][] path, double tempAutoTicks){
        boolean isFinished = false;

        if((currentStep < path.length)){

            double distance;

            int currentX = path[currentStep][0];
            int currentY = path[currentStep][1];
            int currentRot = path[currentStep][2];
            int nextX, nextY, nextRot;

            if(currentStep+1 < path.length){
                nextX = path[currentStep+1][0];
                nextY = path[currentStep+1][1];
                nextRot = path[currentStep+1][2];
            }else{
                nextX = path[currentStep][0];
                nextY = path[currentStep][1];
                nextRot = path[currentStep][2];
            }
            
            double x, y, rotation;
            x = Math.abs(nextX-currentX);
            y = Math.abs(nextY-currentY);

            if(x > 0.5 || x < -0.5){
                x = Math.copySign(0.5, x);
            }else{
                x = x;
            }
            if(y > 0.5 || y < -0.5){
                y = Math.copySign(0.5, y);
            }else{
                y = y;
            }
            

            distance = Math.sqrt(Math.pow(x, 2)+Math.pow(y, 2));
            

            if(Math.abs(tempAutoTicks - drivetrain.getFRDEnc().getPosition()) < inchesToTicks(distance)){
                drivetrain.drive(new Translation2d(x, y), 0, true);
            }else{
                drivetrain.drive(new Translation2d(0,0), 0, true);
                isFinished = true;
            }
            
        }else{
            isFinished = true;
        }
        return isFinished;
    }

    public void setInAuto(boolean in){
        inAuto = in;
    }

    public boolean inAuto(){
        return inAuto;
    }


}
