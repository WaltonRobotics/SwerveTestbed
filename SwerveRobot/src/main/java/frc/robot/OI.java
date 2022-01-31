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
import static frc.robot.utils.Gamepad.Button.*;

public class OI {

    public static Gamepad gamepad = new Gamepad(0);
    public static Gamepad manipulationGamepad = new Gamepad(1);
    public static JoystickButton intakeButton = new JoystickButton(manipulationGamepad, RIGHT_TRIGGER.getIndex());
    public static JoystickButton outtakeButton = new JoystickButton(manipulationGamepad, RIGHT_BUMPER.getIndex());
    public static JoystickButton resetDrivetrainButton = new JoystickButton(gamepad, LEFT_TRIGGER.getIndex());

    static {
        resetDrivetrainButton.whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.reset()),
                new InstantCommand(() -> System.out.println("Reset drivetrain"))
        ));
    }

}
