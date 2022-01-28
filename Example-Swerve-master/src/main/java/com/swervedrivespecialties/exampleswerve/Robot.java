package com.swervedrivespecialties.exampleswerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervedrivespecialties.exampleswerve.autonomous.AutoUtilities;
import com.swervedrivespecialties.exampleswerve.autonomous.BarrelRacing;
import com.swervedrivespecialties.exampleswerve.autonomous.BasicAuto;
import com.swervedrivespecialties.exampleswerve.autonomous.Bounce;
import com.swervedrivespecialties.exampleswerve.autonomous.GalacticSearch;
import com.swervedrivespecialties.exampleswerve.autonomous.Kettering3ballShoot;
import com.swervedrivespecialties.exampleswerve.autonomous.Slalom;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.IntakeIndexSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.ShooterSubsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    private static OI oi;

    private static final String kBarrelRaceAuto = "Barrel Racing";
    private static final String kSlalomAuto = "Slalom";
    private static final String kBounceAuto = "Bounce";
    private static final String kSearchAuto = "Galactic Search";
    private static final String k3ballAuto = "Basic Auto";
    private static final String kBasicAuto = "3 Ball Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private static DrivetrainSubsystem drivetrain;
    private static IntakeIndexSubsystem intakeIndex;
    private static ShooterSubsystem shooter;

    private Compressor c;
    public DoubleSolenoid climb;

    private Servo visionServo;
    private CANSparkMax climbMotor;

    public NetworkTable lime;

    private AutoUtilities util = new AutoUtilities();

    private BarrelRacing barrelAuto;
    private Slalom slalomAuto;
    private Bounce bounceAuto;
    private GalacticSearch searchAuto;
    private Kettering3ballShoot shootAuto;
    private BasicAuto basicAuto;

    private double teleServoPos = 0.129;
    private double driveCamPos = 0.92;
    private double autoServoPos = 1;

    private double zone = 0;

    public static OI getOi() {
        return oi;
    }

    @Override
    public void robotInit() {
        oi = new OI();
        drivetrain = DrivetrainSubsystem.getInstance();
        intakeIndex = IntakeIndexSubsystem.getInstance();
        shooter = ShooterSubsystem.getInstance();

        visionServo = new Servo(RobotMap.VISION_SERVO);
        visionServo.setAngle(16);

        climbMotor = new CANSparkMax(RobotMap.CLIMB_MOTOR, MotorType.kBrushless);

        c = new Compressor(RobotMap.COMPRESSOR);
        c.setClosedLoopControl(true);
        climb = new DoubleSolenoid(RobotMap.COMPRESSOR, 0, 2);
        climb.set(Value.kReverse);
        lime = NetworkTableInstance.getDefault().getTable("limelight");
        // lime.getEntry("camMode").setNumber(0);
        // lime.getEntry("pipeline").setNumber(1);
        // lime.getEntry("ledMode").setNumber(0);
        /*
         * try { logFile = new File("/U/LogFile.txt"); w = new
         * FileWriter("/U/LogFile.txt"); w.
         * write("Main Time, ShooterSpeed/Left, ShooterSpeed/Right, ShooterCurrent/Left, ShooterCurrent/Right, Swerve/FRS, Swerve/FRD, Swerve/FLS, Swerve/FLD, Swerve/BRS, Swerve/BRD, Swerve/BLS, Swerve/BLD\n"
         * ); } catch (IOException e) { e.printStackTrace(); }
         */

        // ahrs = new AHRS(SPI.Port.kMXP);

        drivetrain.getMotor("FRD").setIdleMode(IdleMode.kBrake);
        drivetrain.getMotor("FLD").setIdleMode(IdleMode.kBrake);
        drivetrain.getMotor("BRD").setIdleMode(IdleMode.kBrake);
        drivetrain.getMotor("BLD").setIdleMode(IdleMode.kBrake);

        /*
        m_chooser.setDefaultOption("Barrel Race", kBarrelRaceAuto);
        m_chooser.addOption("Slalom", kSlalomAuto);
        m_chooser.addOption("Bounce", kBounceAuto);
        m_chooser.addOption("Galactic Search", kSearchAuto);
        */
        m_chooser.addOption("Basic Auto", k3ballAuto);
        m_chooser.addOption("3 Ball Auto", kBasicAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    @Override
    public void autonomousInit() {
        // TODO Auto-generated method stub
        m_autoSelected = m_chooser.getSelected();

        super.autonomousInit();

        barrelAuto = new BarrelRacing(drivetrain, shooter, intakeIndex);
        slalomAuto = new Slalom(drivetrain, shooter, intakeIndex);
        bounceAuto = new Bounce(drivetrain, shooter, intakeIndex);
        searchAuto = new GalacticSearch(drivetrain, shooter, intakeIndex);
        shootAuto = new Kettering3ballShoot(drivetrain, shooter, intakeIndex);
        basicAuto = new BasicAuto(drivetrain, shooter, intakeIndex);

        lime.getEntry("camMode").setNumber(1);
        // lime.getEntry("pipeline").setNumber(1);
        // lime.getEntry("ledMode").setNumber(0);

         visionServo.setAngle(16);
    }

    @Override
    public void autonomousPeriodic() {
        // TODO Auto-generated method stub
        super.autonomousPeriodic();
        lime.getEntry("camMode").setNumber(0);
        lime.getEntry("pipeline").setNumber(0);
        lime.getEntry("ledMode").setNumber(3);
        visionServo.setAngle(16);

        switch (m_autoSelected) {
            case kBarrelRaceAuto:
                barrelAuto.runAuto();
                break;
            case kSlalomAuto:
                slalomAuto.runAuto();
                break;
            case kBounceAuto:
                bounceAuto.runAuto();
                break;
            case kSearchAuto:
                if (visionServo.getAngle() == 180) {
                    if (!searchAuto.ballCheckFinished()) {
                        searchAuto.checkBalls();
                        intakeIndex.Index();
                    } else {
                        searchAuto.runAuto();
                        intakeIndex.Index();
                    }
                }

                break;
            case kBasicAuto:
                shootAuto.runAuto();

            case k3ballAuto:
                basicAuto.runAuto();
                break;

            default:
                // Put default auto code here
                break;
        }

        SmartDashboard.putNumber("FRD Enc", drivetrain.getFRDEnc().getPosition());

    }

    @Override
    public void robotPeriodic() {
        intakeIndex.updateSwitches();
        shooter.setAutoShoot(0);
        intakeIndex.setAutoStatus(false);
        Scheduler.getInstance().run();
        SmartDashboard.putNumber("FLD Ticks", drivetrain.getFRDEnc().getPosition());

        if (getOi().getSpecialJoystick().getPOV() == 0) {
            if (teleServoPos <= 1)
                teleServoPos += 0.01;
        } else if (getOi().getSpecialJoystick().getPOV() == 180) {
            if (teleServoPos >= 0)
                teleServoPos -= 0.01;
        }

        if (getOi().getSpecialJoystick().getRawButton(6)) {
            lime.getEntry("camMode").setNumber(0);
            lime.getEntry("pipeline").setNumber(0);
            lime.getEntry("ledMode").setNumber(0);
            visionServo.set(teleServoPos);
        } else if(getOi().getSpecialJoystick().getRawButtonReleased(6)) {
            lime.getEntry("camMode").setNumber(0);
            lime.getEntry("pipeline").setNumber(0);
            lime.getEntry("ledMode").setNumber(0);
            visionServo.set(1);
        }

        SmartDashboard.putNumber("Servo Pos", visionServo.getAngle());

        SmartDashboard.putNumber("TestNumber", util.getTurnCorrect(0, drivetrain.getGyroAngle()));

        double x = lime.getEntry("tlong").getDouble(0);
        double distance = 0.0052 * Math.pow(x, 2) - 3.0557 * x + 508.09;
        SmartDashboard.putNumber("Predicted In", distance);

        zone = 0;

        if (distance > 30 && distance < 80) {
            zone = 1;
        } else if (distance > 80 && distance < 150) {
            zone = 2;
        } else if (distance > 150 && distance < 210) {
            zone = 3;
        } else if (distance > 210 && distance < 270) {
            zone = 4;
        }

        SmartDashboard.putNumber("Zone", zone);

        /*
         * if(Robot.getOi().getSpecialJoystick().getRawButton(7)){
         * lime.getEntry("camMode").setNumber(0);
         * lime.getEntry("pipeline").setNumber(0);
         * lime.getEntry("ledMode").setNumber(3); }else
         * if(Robot.getOi().getSpecialJoystick().getRawButton(8)){
         * lime.getEntry("camMode").setNumber(0);
         * lime.getEntry("pipeline").setNumber(1);
         * lime.getEntry("ledMode").setNumber(1); }else{
         * lime.getEntry("camMode").setNumber(1);
         * lime.getEntry("pipeline").setNumber(0);
         * lime.getEntry("ledMode").setNumber(1); } /*
         * 
         * RS = shooter.getRightVelocity(); LS = shooter.getLeftVelocity(); RC =
         * shooter.getRightCurrent(); LC = shooter.getLeftCurrent();
         * 
         * FRS = drivetrain.getMotor('F','R','S').getOutputCurrent(); FRD =
         * drivetrain.getMotor('F','R','D').getOutputCurrent(); FLS =
         * drivetrain.getMotor('F','L','S').getOutputCurrent(); FLD =
         * drivetrain.getMotor('F','L','D').getOutputCurrent(); BRS =
         * drivetrain.getMotor('B','R','S').getOutputCurrent(); BRD =
         * drivetrain.getMotor('B','R','D').getOutputCurrent(); BLS =
         * drivetrain.getMotor('B','L','S').getOutputCurrent(); BLD =
         * drivetrain.getMotor('B','L','D').getOutputCurrent();
         * 
         * 
         * currentTime = Timer.getFPGATimestamp();
         * 
         * 
         * try { w.write(currentTime + "," + LS + "," + RS + "," + LC + "," + RC + "," +
         * FRS + "," + FRD + "," + FLS + "," + FLD + "," + BRS + "," + BRD + "," + BLS +
         * "," + BLD + "\n");
         * 
         * } catch (IOException e) { e.printStackTrace(); }
         * 
         */

        if (getOi().getPrimaryJoystick().getRawButtonPressed(1)) {
            if (climb.get() == Value.kForward) {
                climb.set(Value.kReverse);
            } else {
                climb.set(Value.kForward);
            }
        }

        if (getOi().getSpecialJoystick().getPOV() == 90) {
            //.climbMotor.set(1);
        } else if (getOi().getSpecialJoystick().getPOV() == 270) {
            climbMotor.set(-0.5);
        } else {
            climbMotor.set(0);
        }

    }

}
