package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.org.strykeforce.swerve.SwerveModule;

import static frc.robot.Constants.SmartDashboardKeys.DRIVETRAIN_SETPOINT_ANGLE_DEGREES;
import static frc.robot.Robot.drivetrain;

public class RotateModulesToAngle extends CommandBase {

    public RotateModulesToAngle() {
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        for (SwerveModule module : drivetrain.getSwerveModules()) {
            SwerveModuleState desiredState = new SwerveModuleState(
                    0.0,
                    Rotation2d.fromDegrees(SmartDashboard.getNumber(DRIVETRAIN_SETPOINT_ANGLE_DEGREES, 0.0)
            ));

            module.setDesiredState(desiredState);
        }
    }

}
