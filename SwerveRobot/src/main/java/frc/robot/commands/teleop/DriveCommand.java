package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.org.strykeforce.thirdcoast.util.ExpoScale;

import static frc.robot.Constants.SwerveDriveConfig.kMaxOmega;
import static frc.robot.Constants.SwerveDriveConfig.kMaxSpeedMetersPerSecond;
import static frc.robot.OI.gamepad;
import static frc.robot.Robot.drivetrain;

public class DriveCommand extends CommandBase {

    private static final double FORWARD_DEADBAND = 0.05;
    private static final double STRAFE_DEADBAND = 0.05;
    private static final double YAW_DEADBAND = 0.01;

    private static final double FORWARD_XPOSCALE = 0.6;
    private static final double STRAFE_XPOSCALE = 0.6;
    private static final double YAW_XPOSCALE = 0.75;

    private final ExpoScale forwardScale;
    private final ExpoScale strafeScale;
    private final ExpoScale yawScale;

    public DriveCommand() {
        addRequirements(drivetrain);

        forwardScale = new ExpoScale(FORWARD_DEADBAND, FORWARD_XPOSCALE);
        strafeScale = new ExpoScale(STRAFE_DEADBAND, STRAFE_XPOSCALE);
        yawScale = new ExpoScale(YAW_DEADBAND, YAW_XPOSCALE);
    }

    @Override
    public void execute() {
        double forward = forwardScale.apply(-gamepad.getLeftY());
        double strafe = strafeScale.apply(-gamepad.getLeftX());
        double yaw = yawScale.apply(-gamepad.getRightX());
        double vx = forward * kMaxSpeedMetersPerSecond;
        double vy = strafe * kMaxSpeedMetersPerSecond;
        double omega = yaw * kMaxOmega;

        drivetrain.move(vx, vy, omega, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0);
    }

}
