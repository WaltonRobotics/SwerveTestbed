package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.utils.Gamepad;

import static frc.robot.Constants.InputDevices.*;
import static frc.robot.Robot.drivetrain;
import static frc.robot.utils.Gamepad.Button.LEFT_TRIGGER;

public class OI {

    public static Joystick leftJoystick = new Joystick(kLeftJoystickPort);
    public static Joystick rightJoystick = new Joystick(kRightJoystickPort);
    public static Gamepad gamepad = new Gamepad(kGamepadPort);

    public static JoystickButton resetDrivetrainButton = new JoystickButton(gamepad, LEFT_TRIGGER.getIndex());

    static {
        resetDrivetrainButton.whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.reset()),
                new InstantCommand(() -> System.out.println("Reset drivetrain"))
        ));
    }

}
