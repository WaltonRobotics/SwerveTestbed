package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.SwerveDriveConfig.kMaxSpeedMetersPerSecond;
import static frc.robot.Robot.drivetrain;

public class AutoDrive extends CommandBase {

    public AutoDrive() {
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.move(-kMaxSpeedMetersPerSecond * 0.25, 0.0, 0.0, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
