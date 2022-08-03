package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.Constants.SmartDashboardKeys.DRIVETRAIN_SETPOINT_VELOCITY;
import static frc.robot.Robot.drivetrain;

public class SetModulesToVelocity extends SequentialCommandGroup {

    public SetModulesToVelocity() {
        addRequirements(drivetrain);
        addCommands(
                new RunCommand(() -> {
                    drivetrain.setSpeed(SmartDashboard.getNumber(DRIVETRAIN_SETPOINT_VELOCITY, 0.0));
                })
        );
    }

}
