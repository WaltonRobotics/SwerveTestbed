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
                        drivetrain.move(kMaxSpeedMetersPerSecond / 4.0, 0.0, 0.0, false),
                        drivetrain).withTimeout(5.0)
        );
    }

}
