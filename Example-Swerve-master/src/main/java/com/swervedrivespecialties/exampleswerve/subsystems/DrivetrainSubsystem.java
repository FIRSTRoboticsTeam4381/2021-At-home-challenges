package com.swervedrivespecialties.exampleswerve.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkMax.IdleMode;
import com.swervedrivespecialties.exampleswerve.RobotMap;
import com.swervedrivespecialties.exampleswerve.commands.DriveCommand;
import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;
import org.frcteam2910.common.robot.drivers.NavX;


public class DrivetrainSubsystem extends Subsystem {
    private static final double TRACKWIDTH = 23.5;
    private static final double WHEELBASE = 23.5;

    private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(321+180);
    private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(218);
    private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(62+180);
    private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(271);

    private CANSparkMax frontLeftAngleMotor = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax frontRightAngleMotor = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax backLeftAngleMotor = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax backRightAngleMotor = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    private CANSparkMax frontLeftDriveMotor = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax frontRightDriveMotor = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax backLeftDriveMotor = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax backRightDriveMotor = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    private static DrivetrainSubsystem instance;

    private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new CANAnalog(frontLeftAngleMotor, AnalogMode.kAbsolute), FRONT_LEFT_ANGLE_OFFSET)
            .angleMotor(frontLeftAngleMotor,
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(frontLeftDriveMotor,
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new CANAnalog(frontRightAngleMotor, AnalogMode.kAbsolute), FRONT_RIGHT_ANGLE_OFFSET)
            .angleMotor(frontRightAngleMotor,
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(frontRightDriveMotor,
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new CANAnalog(backLeftAngleMotor, AnalogMode.kAbsolute), BACK_LEFT_ANGLE_OFFSET)
            .angleMotor(backLeftAngleMotor,
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(backLeftDriveMotor,
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new CANAnalog(backRightAngleMotor, AnalogMode.kAbsolute), BACK_RIGHT_ANGLE_OFFSET)
            .angleMotor(backRightAngleMotor,
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(backRightDriveMotor,
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)
    );

    private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);

    public double getGyroAngle(){
        return gyroscope.getAngle().toDegrees();
    }
    

    public DrivetrainSubsystem() {
        gyroscope.calibrate();
        gyroscope.setInverted(true); // You might not need to invert the gyro
        gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle());
        frontLeftAngleMotor.setIdleMode(IdleMode.kCoast);

        frontLeftModule.setName("Front Left");
        frontRightModule.setName("Front Right");
        backLeftModule.setName("Back Left");
        backRightModule.setName("Back Right");
    }

    public static DrivetrainSubsystem getInstance() {
        if (instance == null) {
            instance = new DrivetrainSubsystem();
        }

        return instance;
    }

    @Override
    public void periodic() {
        frontLeftModule.updateSensors();
        frontRightModule.updateSensors();
        backLeftModule.updateSensors();
        backRightModule.updateSensors();

        SmartDashboard.putNumber("Front Left Module Angle", Math.toDegrees(frontLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Front Right Module Angle", Math.toDegrees(frontRightModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Left Module Angle", Math.toDegrees(backLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Right Module Angle", Math.toDegrees(backRightModule.getCurrentAngle()));

        SmartDashboard.putNumber("Gyroscope Angle", gyroscope.getAngle().toDegrees());

        frontLeftModule.updateState(TimedRobot.kDefaultPeriod);
        frontRightModule.updateState(TimedRobot.kDefaultPeriod);
        backLeftModule.updateState(TimedRobot.kDefaultPeriod);
        backRightModule.updateState(TimedRobot.kDefaultPeriod);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
        rotation *= 2.0 / Math.hypot(WHEELBASE, TRACKWIDTH);
        ChassisSpeeds speeds;
        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                    Rotation2d.fromDegrees(gyroscope.getAngle().toDegrees()));
        } else {
            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        frontLeftModule.setTargetVelocity(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        frontRightModule.setTargetVelocity(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        backLeftModule.setTargetVelocity(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        backRightModule.setTargetVelocity(states[3].speedMetersPerSecond, states[3].angle.getRadians());
    }

    public void resetGyroscope() {
        gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle());
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveCommand());
    }

    public CANEncoder getFRDEnc(){
            return frontRightDriveMotor.getEncoder();
    }

    
    public CANSparkMax getMotor(String motorName){
            CANSparkMax motor = frontRightAngleMotor;
            int motorNum = 0;

            char motorNameArr[] = motorName.toCharArray();

            char frontBack = motorNameArr[0];
            char leftRight = motorNameArr[1];
            char driveSteer = motorNameArr[2];

            if(frontBack == 'F'){
                    motorNum += 200;
            }else{
                    motorNum += 100;
            }
            if(leftRight == 'L'){
                    motorNum += 20;
            }else{
                    motorNum += 10;
            }
            if(driveSteer == 'D'){
                    motorNum += 2;
            }else{
                    motorNum += 1;
            }


            switch(motorNum){
                    case 222: motor = frontLeftDriveMotor;
                    break;
                    case 221: motor = frontLeftAngleMotor;
                    break;
                    case 212: motor = frontRightDriveMotor;
                    break;
                    case 211: motor = frontRightAngleMotor;
                    break;
                    case 122: motor = backLeftDriveMotor;
                    break;
                    case 121: motor = backLeftAngleMotor;
                    break;
                    case 112: motor = backRightDriveMotor;
                    break;
                    case 111: motor = backRightAngleMotor;
                    break;
            }
            

            return motor;
    }
    
}
