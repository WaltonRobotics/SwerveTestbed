package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.utils.Gamepad;

import static frc.robot.Constants.InputDevices.*;

public class OI {

    public static Joystick leftJoystick = new Joystick(kLeftJoystickPort);
    public static Joystick rightJoystick = new Joystick(kRightJoystickPort);
    public static Gamepad gamepad = new Gamepad(kGamepadPort);

}
