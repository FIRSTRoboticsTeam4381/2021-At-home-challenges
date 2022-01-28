package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class OI {
    /*
       Add your joysticks and buttons here
     */
    private Joystick primaryJoystick = new Joystick(0);
    private Joystick specialJoystick = new Joystick(1);

    //{ps4}{xbx}{x3d}
    //{ForwardAxis, StrafeAxis, TurnAxis, GyroButton, ThrottleAxis, PickupButton}
    private int[][] schemes = {{1, 0, 2, 10, 4, 5}, {1, 0, 4, 8, 3, 5}, {1, 0, 2, 5, 3, 2}};
    private int schemeNum = 0;

    public OI() {
        // Back button zeroes the drivetrain
        new JoystickButton(primaryJoystick, getControlNum('g')).whenPressed(
                new InstantCommand(() -> DrivetrainSubsystem.getInstance().resetGyroscope())
        );
    }

    public Joystick getPrimaryJoystick() {
        return primaryJoystick;
    }

    public Joystick getSpecialJoystick() {
        return specialJoystick;
    }

    public void setControlScheme(int scheme){
        switch(scheme){
            case 0:
                schemeNum = 0;
                break;
            case 1:
                schemeNum = 0;
                break;
            case 2:
                schemeNum = 0;
                break;
            default:
                schemeNum = 0;
        }
    }
    /*f = forward
    * s = strafe
    * t = turn
    * p = pickup
    * g = gyro
    * x = throttle
    */
    public int getControlNum(char designation){
        int num = 0;
        
        switch(designation){
            case 'f':
                num = schemes[schemeNum][0];
                break;
            case 's':
                num = schemes[schemeNum][1];
                break;
            case 't':
                num = schemes[schemeNum][2];
                break;
            case 'p':
                num = schemes[schemeNum][5];
                break;
            case 'g':
                num = schemes[schemeNum][3];
                break;
            case 'x':
                num = schemes[schemeNum][4];
                break;
        }

        return num;
    }
}
