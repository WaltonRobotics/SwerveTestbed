package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.utils.Gamepad;

import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftTrigger;
import static edu.wpi.first.wpilibj.XboxController.Axis.kRightTrigger;
import static edu.wpi.first.wpilibj.XboxController.Button.kRightBumper;
import static frc.robot.Constants.InputDevices.*;
import static frc.robot.Robot.drivetrain;
import static frc.robot.utils.Gamepad.Button.*;

public class OI {

    public static XboxController gamepad = new XboxController(kLeftControllerPort);
    public static XboxController manipulationGamepad = new XboxController(kRightControllerPort);
    public static JoystickButton intakeButton = new JoystickButton(manipulationGamepad, kRightTrigger.value);
    public static JoystickButton outtakeButton = new JoystickButton(manipulationGamepad, kRightBumper.value);
    public static JoystickButton resetDrivetrainButton = new JoystickButton(gamepad, kLeftTrigger.value);

    static {
        resetDrivetrainButton.whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.reset()),
                new InstantCommand(() -> System.out.println("Reset drivetrain"))
        ));
    }

}
