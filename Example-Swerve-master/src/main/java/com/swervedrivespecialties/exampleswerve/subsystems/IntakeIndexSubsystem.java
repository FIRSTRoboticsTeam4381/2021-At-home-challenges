package com.swervedrivespecialties.exampleswerve.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervedrivespecialties.exampleswerve.RobotMap;
import com.swervedrivespecialties.exampleswerve.commands.IntakeIndexCommand;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeIndexSubsystem extends Subsystem{

    private static IntakeIndexSubsystem instance;
    private CANSparkMax Intake;
    private CANSparkMax Index;

    private DigitalInput BreakBeam, Switch1, Switch2, Switch3;

    private boolean indexing = false;
    
    public boolean autoShoot = false;

    private double numBalls = 0;

    private boolean[] switchesArray = {false, false, false, false};
    private int state = 999;

    private double intakeSpeed = 0.75;


    public IntakeIndexSubsystem() {
        Intake = new CANSparkMax(RobotMap.INTAKE_MOTOR, MotorType.kBrushless);
        Index = new CANSparkMax(RobotMap.INDEXER_MOTOR, MotorType.kBrushless);

        BreakBeam = new DigitalInput(0);
        Switch1 = new DigitalInput(1);
        Switch2 = new DigitalInput(2);
        Switch3 = new DigitalInput(3);


    }

    public static IntakeIndexSubsystem getInstance() {
        if(instance == null) {
            instance = new IntakeIndexSubsystem();
        }

        return instance;
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("BreakBeam", BreakBeam.get());
        SmartDashboard.putBoolean("Switch 1", Switch1.get());
        SmartDashboard.putBoolean("Switch 2", Switch2.get());
        SmartDashboard.putBoolean("Switch 3", Switch3.get());
        SmartDashboard.putNumber("Number of Balls", numBalls);

        updateSwitches();
        SmartDashboard.putNumber("Case", getCase());
        SmartDashboard.putNumber("State", state);
    }

    public void updateSwitches(){
        switchesArray[0] = !BreakBeam.get();
        switchesArray[1] = !Switch1.get();
        switchesArray[2] = !Switch2.get();
        switchesArray[3] = !Switch3.get();
    }

    public int getCase(){
        int integerCase = 
            (switchesArray[0] ? 1:0) + 
            (switchesArray[1] ? 2:0) +
            (switchesArray[2] ? 4:0) +
            (switchesArray[3] ? 8:0);    
        return integerCase;
    }

    public void Index(){
        
        switch(state){
            case 0:
                Intake.set(intakeSpeed);
                state = getCase();
                break;
            case 1:
                if(runUntil(1, true)){
                    state = getCase();
                }else{
                    state = 1;
                }
                break;
            case 3:
                if(runUntil(2, true)){
                    state = getCase();
                }else{
                    state = 3;
                }
                break;
            case 5:
                state = 7;
                break;
            case 7:
                if(runUntil(3, true)){
                    Intake.set(0);
                    state = getCase();
                }else{
                    state = 7;
                }
                break;
            case 8:
                Intake.set(0);
                break;
            default:
                Intake.set(intakeSpeed);
                Index.set(0);
                state = getCase();
        
        }
        
    }

    public void FireBalls(double desiredRPM, double actualRPM){
        if(actualRPM > desiredRPM - 20){
            Index.set(-0.5);
        }else{
            Index.set(0);
        }
        state = getCase();
    }

    private boolean runUntil(int button, boolean desired){
        boolean isfinished;
        if(switchesArray[button] == desired){
            Index.set(0);
            Intake.set(intakeSpeed);
            isfinished = true;
        }else{
            Index.set(-0.3);
            Intake.set(0);
            isfinished = false;
        }

        return isfinished;
    }

    public void reverse(){
        Intake.set(-0.4);
        Index.set(0.4);
        state = getCase();
    }

    public void rawIntake(double speed){  
        Intake.set(speed);
    }

    public void rawIndex(double speed){
        Index.set(speed);
    }

    public boolean autoStatus(){
        return autoShoot;
    }
    public void setAutoStatus(boolean shootTemp){
        autoShoot = shootTemp;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new IntakeIndexCommand());
    }
}
