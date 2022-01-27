package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.SwerveDriveConfig.kMaxOmega;
import static frc.robot.Robot.drivetrain;

public class SwerveTrajectoryCommand extends CommandBase {

    private final PathPlannerTrajectory trajectory;
    private final Timer timer = new Timer();
    private HolonomicDriveController holonomicDriveController;

    public SwerveTrajectoryCommand(PathPlannerTrajectory trajectory) {
        addRequirements(drivetrain);
        this.trajectory = trajectory;
    }

    public void initialize() {
        var p = 6.0;
        var d = p / 100.0;
        ProfiledPIDController thetaController =
                new ProfiledPIDController(
                        3.0,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(kMaxOmega / 2.0, 3.14));
        thetaController.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
        holonomicDriveController =
                new HolonomicDriveController(
                        new PIDController(p, 0, d), new PIDController(p, 0, d), thetaController);

        holonomicDriveController.setEnabled(true);

        timer.reset();
        timer.start();

        LiveDashboardTable.getInstance().setFollowingPath(true);

        LiveDashboardHelper.putRobotData(drivetrain.getPoseMeters());
        LiveDashboardHelper.putTrajectoryData(trajectory.getInitialPose());

        drivetrain.getField().getObject("traj").setTrajectory(trajectory);
    }

    public void execute() {
        double currentTime = timer.get();

        PathPlannerTrajectory.PathPlannerState state = (PathPlannerTrajectory.PathPlannerState) trajectory.sample(currentTime);
        ChassisSpeeds speeds = holonomicDriveController.calculate(drivetrain.getPoseMeters(), state, state.holonomicRotation);

        drivetrain.move(
                speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);

        LiveDashboardHelper.putRobotData(drivetrain.getPoseMeters());
        LiveDashboardHelper.putTrajectoryData(trajectory.sample(currentTime).poseMeters);
    }
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        drivetrain.drive(0.0, 0.0, 0.0);
    }
}
