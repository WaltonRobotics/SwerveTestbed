package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.Constants.SwerveDriveConfig.kMaxSpeedMetersPerSecond;
import static frc.robot.Robot.drivetrain;

public class AutoDrive extends SequentialCommandGroup {

    public AutoDrive() {
        addCommands(
                new RunCommand(() ->
                        drivetrain.move(-kMaxSpeedMetersPerSecond * 0.25, 0.0, 0.0, true),
                        drivetrain).withTimeout(5.0)
        );
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
