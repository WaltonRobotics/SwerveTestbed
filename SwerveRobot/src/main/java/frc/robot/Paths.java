package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.util.Units;


public class Paths {

    public static PathPlannerTrajectory testTrajectory = PathPlanner.loadPath("TestTrajectory", Units.feetToMeters(11.0), Units.feetToMeters(5.0));

}
