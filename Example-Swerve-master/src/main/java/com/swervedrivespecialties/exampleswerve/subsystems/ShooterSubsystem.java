package com.swervedrivespecialties.exampleswerve.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.RobotMap;
import com.swervedrivespecialties.exampleswerve.commands.ShooterCommand;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends Subsystem{

    private static ShooterSubsystem instance;

    private CANSparkMax LeftShootMotor;
    private CANSparkMax RightShootMotor;
    private CANEncoder LeftShootEnc;
    private CANEncoder RightShootEnc;

    private CANPIDController PIDController;
    private CANPIDController RightPIDController;

    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private double maxRPM, maxVel, minVel, maxAcc, allowedErr;

    private boolean One = false;
    private boolean Two = false;
    private boolean Three = false;
    private boolean Four = false;
    private boolean Five = false;

    private double SETPOINT = 0;
    
    private boolean autoShoot;


    public ShooterSubsystem() {
        LeftShootMotor = new CANSparkMax(RobotMap.SHOOTER_LEFT_MOTOR, MotorType.kBrushless);
        RightShootMotor = new CANSparkMax(RobotMap.SHOOTER_RIGHT_MOTOR, MotorType.kBrushless);
        LeftShootEnc = new CANEncoder(LeftShootMotor);
        RightShootEnc = new CANEncoder(RightShootMotor);

        RightShootMotor.follow(LeftShootMotor, true);

        PIDController = LeftShootMotor.getPIDController();

        kP = 0.0003;
        kI = 0;
        kD = 0.0005;
        kIz = 0;
        kFF = 0.00019231;
        kMaxOutput = 1;
        kMinOutput = -1;

        maxRPM = 5200;

        PIDController.setP(kP);
        PIDController.setI(kI);
        PIDController.setD(kD);
        PIDController.setIZone(kIz);
        PIDController.setFF(kFF);
        PIDController.setOutputRange(kMinOutput, kMaxOutput);
        

        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);

        SmartDashboard.putNumber("Speed", 0);

    }

    public static ShooterSubsystem getInstance() {
        if(instance == null) {
            instance = new ShooterSubsystem();
        }

        return instance;
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("LeftMotorSpeed", -LeftShootEnc.getVelocity());
        SmartDashboard.putNumber("RightMotorSpeed", RightShootEnc.getVelocity());

        if(RightShootEnc.getVelocity() > 1000){
            One = true;
        }else{
            One = false;
        }
        if(RightShootEnc.getVelocity() > 2000){
            Two = true;
        }else{
            Two = false;
        }
        if(RightShootEnc.getVelocity() > 3000){
            Three = true;
        }else{
            Three = false;
        }
        if(RightShootEnc.getVelocity() > 4000){
            Four = true;
        }else{
            Four = false;
        }
        if(RightShootEnc.getVelocity() > 5000){
            Five = true;
        }else{
            Five = false;
        }

        SmartDashboard.putBoolean("1000 RPM Motor, 2000 RPM Wheel", One);
        SmartDashboard.putBoolean("2000 RPM Motor, 4000 RPM Wheel", Two);
        SmartDashboard.putBoolean("3000 RPM Motor, 6000 RPM Wheel", Three);
        SmartDashboard.putBoolean("4000 RPM Motor, 8000 RPM Wheel", Four);
        SmartDashboard.putBoolean("5000 RPM Motor, 10000 RPM Wheel", Five);
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);


        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);


        if((p != kP)) { PIDController.setP(p); kP = p; }
        if((i != kI)) { PIDController.setI(i); kI = i; }
        if((d != kD)) { PIDController.setD(d); kD = d; }
        if((iz != kIz)) { PIDController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { PIDController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
        PIDController.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
        }




        //double setPoint = SETPOINT*maxRPM;
        double setPoint = SETPOINT;
        PIDController.setReference(setPoint, ControlType.kVelocity);
    
        SmartDashboard.putNumber("SetPoint", setPoint);
        
    }

    public void shoot(double speed){
        SETPOINT = speed;
        
    }

    public boolean getAutoShoot(){
        return autoShoot;
    }
    public void setAutoShoot(int a){
        if(a == 1){
            autoShoot = true;
        }else{
            autoShoot = false;
        }
    }

    public double getLeftVelocity(){
        return -LeftShootEnc.getVelocity();
    }
    public double getRightVelocity(){
        return RightShootEnc.getVelocity();
    }
    public double getLeftCurrent(){
        return LeftShootMotor.getOutputCurrent();
    }
    public double getRightCurrent(){
        return RightShootMotor.getOutputCurrent();
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new ShooterCommand());
    }
}
