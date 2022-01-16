package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;

import java.util.Arrays;

import static frc.robot.Robot.drivetrain;

public class Paths {

    public static Trajectory sCurve = generateSCurveTrajectory();
    public static Trajectory straight = generateStraightTrajectory();
    public static Trajectory backupToOrigin = generateBackupToOrigin();

    public static Trajectory generateSCurveTrajectory() {
        TrajectoryConfig config = new TrajectoryConfig(
                Units.feetToMeters(11.0), Units.feetToMeters(7.0));

        config.setKinematics(drivetrain.getSwerveDriveKinematics());

        return TrajectoryGenerator.generateTrajectory(
                Arrays.asList(
                        new Pose2d(Units.feetToMeters(2.5), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0)),
                        new Pose2d(Units.feetToMeters(10.461), Units.feetToMeters(16.273), Rotation2d.fromDegrees(0))),
                config
        );
    }

    public static Trajectory generateStraightTrajectory() {
        TrajectoryConfig config = new TrajectoryConfig(
                Units.feetToMeters(11.0), Units.feetToMeters(7.0));

        config.setKinematics(drivetrain.getSwerveDriveKinematics());

        return TrajectoryGenerator.generateTrajectory(
                Arrays.asList(
                        new Pose2d(Units.feetToMeters(10.461), Units.feetToMeters(16.273), Rotation2d.fromDegrees(0)),
                        new Pose2d(Units.feetToMeters(15.461), Units.feetToMeters(16.273), Rotation2d.fromDegrees(0))),
                config
        );
    }

    public static Trajectory generateBackupToOrigin() {
        TrajectoryConfig config = new TrajectoryConfig(
                Units.feetToMeters(11.0), Units.feetToMeters(7.0));

        config.setKinematics(drivetrain.getSwerveDriveKinematics());
        config.setReversed(true);

        return TrajectoryGenerator.generateTrajectory(
                Arrays.asList(
                        new Pose2d(Units.feetToMeters(15.461), Units.feetToMeters(16.273), Rotation2d.fromDegrees(0)),
                        new Pose2d(Units.feetToMeters(2.5), Units.feetToMeters(7.5), Rotation2d.fromDegrees(0))),
                config
        );
    }

}
