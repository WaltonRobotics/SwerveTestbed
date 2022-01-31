package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.Constants.SmartDashboardKeys.DRIVETRAIN_SETPOINT_ANGLE_DEGREES;
import static frc.robot.Robot.drivetrain;

public class RotateModulesToAngle extends SequentialCommandGroup {

    public RotateModulesToAngle() {
        addRequirements(drivetrain);
        addCommands(
                new RunCommand(() -> {
                    drivetrain.setSpeedAndRotation(0.0, SmartDashboard.getNumber(DRIVETRAIN_SETPOINT_ANGLE_DEGREES, 0.0));
                }).withTimeout(5.0)
        );
    }

}
