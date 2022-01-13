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

    public static Trajectory testTrajectory = generateTestTrajectory();

    public static Trajectory generateTestTrajectory() {
        TrajectoryConfig config = new TrajectoryConfig(
                Units.feetToMeters(4.0), Units.feetToMeters(3.0));

        config.setKinematics(drivetrain.getSwerveDriveKinematics());

        return TrajectoryGenerator.generateTrajectory(
                Arrays.asList(
                        new Pose2d(Units.feetToMeters(7.541), Units.feetToMeters(4.866), Rotation2d.fromDegrees(0)),
                        new Pose2d(Units.feetToMeters(16.396), Units.feetToMeters(8.81), Rotation2d.fromDegrees(0))),
                config
        );
    }

}
