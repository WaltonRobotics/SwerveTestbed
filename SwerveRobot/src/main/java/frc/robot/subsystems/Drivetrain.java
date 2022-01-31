package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.org.strykeforce.swerve.SwerveDrive;
import frc.lib.org.strykeforce.swerve.SwerveModule;
import frc.robot.commands.auton.LiveDashboardHelper;
import frc.robot.utils.WaltonSwerveModule;

import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.Constants.SwerveDriveConfig.*;
import static frc.robot.Robot.drivetrain;

public class Drivetrain extends SubsystemBase {

    private Field2d field = new Field2d();

    private final SwerveDrive swerveDrive;
    private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

    private final PIDController xController = new PIDController(6.0, 0.0, 6.0 / 100.0);
    private final PIDController yController = new PIDController(6.0, 0.0, 6.0 / 100.0);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            3.0,
            0,
            0,
            new TrapezoidProfile.Constraints(kMaxOmega / 2.0, 3.14));

    public Drivetrain() {
        var moduleBuilder =
                new WaltonSwerveModule.Builder()
                        .driveGearRatio(kDriveGearRatio)
                        .wheelDiameterInches(kWheelDiameterInches)
                        .driveMaximumMetersPerSecond(kMaxSpeedMetersPerSecond);

        WaltonSwerveModule[] swerveModules = new WaltonSwerveModule[4];
        Translation2d[] wheelLocations = getWheelLocationMeters();

        boolean[] inversions = {true, true, true, false};

        for (int i = 0; i < 4; i++) {
            var azimuthSparkMax = new CANSparkMax(i + 1, CANSparkMaxLowLevel.MotorType.kBrushless);
            azimuthSparkMax.restoreFactoryDefaults();
            azimuthSparkMax.enableVoltageCompensation(12.0);
            azimuthSparkMax.setSmartCurrentLimit(80);
            azimuthSparkMax.setOpenLoopRampRate(0.0);
            azimuthSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
            azimuthSparkMax.setInverted(true);

            RelativeEncoder azimuthRelativeEncoder = azimuthSparkMax.getEncoder();
            SparkMaxPIDController azimuthPID = azimuthSparkMax.getPIDController();

            double relativeEncoderDegreesPerTick = 1.0 / (5.33 * 12.0);
            double inverseEncoderConstant = 1.0 / relativeEncoderDegreesPerTick;

            azimuthRelativeEncoder.setPositionConversionFactor(relativeEncoderDegreesPerTick);
            azimuthRelativeEncoder.setVelocityConversionFactor(relativeEncoderDegreesPerTick);

            // Smart Motion Configuration
            azimuthPID.setP(5e-5 * inverseEncoderConstant);
            azimuthPID.setI(1e-6 * inverseEncoderConstant);
            azimuthPID.setD(0.0);
            azimuthPID.setIZone(0.0);
            azimuthPID.setFF(0.000156 * inverseEncoderConstant);
            azimuthPID.setOutputRange(-1.0, 1.0);

            /**
             * Smart Motion coefficients are set on a SparkMaxPIDController object
             *
             * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
             * the pid controller in Smart Motion mode
             * - setSmartMotionMinOutputVelocity() will put a lower bound in
             * RPM of the pid controller in Smart Motion mode
             * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
             * of the pid controller in Smart Motion mode
             * - setSmartMotionAllowedClosedLoopError() will set the max allowed
             * error for the pid controller in Smart Motion mode
             */
            int smartMotionSlot = 0;
            azimuthPID.setSmartMotionMaxVelocity(120, smartMotionSlot);
            azimuthPID.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
            azimuthPID.setSmartMotionMaxAccel(100, smartMotionSlot);
            azimuthPID.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);

            var driveTalon = new TalonFX(i + 11);
            driveTalon.configFactoryDefault(kTalonConfigTimeout);
            driveTalon.configAllSettings(getDriveTalonConfig(), kTalonConfigTimeout);
            driveTalon.enableVoltageCompensation(true);
            driveTalon.setNeutralMode(NeutralMode.Brake);

            driveTalon.setInverted(inversions[i]);
            driveTalon.setSensorPhase(inversions[i]);

            DutyCycle encoderPWM = new DutyCycle(new DigitalInput(i));

//            ProfiledPIDController controller = new ProfiledPIDController(
//                    /* 20.0 / 1023.0 */ 10.0 / 4096.0, 0.0, 0.0,
//                    new TrapezoidProfile.Constraints(800 * 10, 1000 * 10)
//            );
//
//            controller.setTolerance(30.0);
//            controller.enableContinuousInput(-90 * 4096.0, 90 * 4096.0);

            swerveModules[i] =
                    moduleBuilder
                            .azimuthSparkMax(azimuthSparkMax)
                            .driveTalon(driveTalon)
                            .azimuthAbsoluteEncoderPWM(encoderPWM)
                            .wheelLocationMeters(wheelLocations[i])
                            .build();

            swerveModules[i].loadAndSetAzimuthZeroReference();
        }

        swerveDrive = new SwerveDrive(ahrs, swerveModules);
        resetHeading();
//        setHeadingOffset(Rotation2d.fromDegrees(180));

        SmartDashboard.putData("Field", field);

//        saveCurrentPositionsAsAzimuthZeros();
    }

//    public void loadAzimuthZeroReference() {
//        for (SwerveModule module : getSwerveModules()) {
//            module.loadAndSetAzimuthZeroReference();
//        }
//    }

    public void saveLeftFrontZero(int absoluteCounts) {
        getSwerveModules()[0].storeAzimuthZeroReference(absoluteCounts);
    }

    public void saveRightFrontZero(int absoluteCounts) {
        getSwerveModules()[1].storeAzimuthZeroReference(absoluteCounts);
    }

    public void saveLeftRearZero(int absoluteCounts) {
        getSwerveModules()[2].storeAzimuthZeroReference(absoluteCounts);
    }

    public void saveRightRearZero(int absoluteCounts) {
        getSwerveModules()[3].storeAzimuthZeroReference(absoluteCounts);
    }

    public void reset() {
        resetDriveEncoders();
        resetHeading();
        reloadAzimuthZeros();
    }

    public Field2d getField() {
        return field;
    }

    /**
     * Returns the swerve drive kinematics object for use during trajectory configuration.
     *
     * @return the configured kinemetics object
     */
    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return swerveDrive.getKinematics();
    }

    public void resetDriveEncoders() {
        swerveDrive.resetDriveEncoders();
    }

    public void reloadAzimuthZeros() {
        swerveDrive.reloadAzimuthZeros();
    }

    /**
     * Returns the configured swerve drive modules.
     */
    public SwerveModule[] getSwerveModules() {
        return swerveDrive.getSwerveModules();
    }

    /**
     * Resets the robot's position on the field.
     *
     * @param pose the current pose
     */
    public void resetPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    /**
     * Returns the position of the robot on the field.
     *
     * @return the pose of the robot (x and y ane in meters)
     */
    public Pose2d getPoseMeters() {
        return swerveDrive.getPoseMeters();
    }

    public void setSpeedAndRotation(double speed, double rotationAngleDegrees) {
        for (SwerveModule module : getSwerveModules()) {
            ((WaltonSwerveModule)module).setDriveClosedLoopMetersPerSecond(speed);
            ((WaltonSwerveModule)module).setAzimuthRotation2d(Rotation2d.fromDegrees(rotationAngleDegrees));
        }
    }

    /**
     * Perform periodic swerve drive odometry update.
     */
    @Override
    public void periodic() {
//        setSpeedAndRotation(0.0, 45.0);

        swerveDrive.periodic();

        field.setRobotPose(getPoseMeters());
        LiveDashboardHelper.putRobotData(getPoseMeters());

        SmartDashboard.putNumber(DRIVETRAIN_LEFT_FRONT_ABSOLUTE_POSITION, ((WaltonSwerveModule)getSwerveModules()[0]).getAzimuthAbsoluteEncoderCounts());
        SmartDashboard.putNumber(DRIVETRAIN_RIGHT_FRONT_ABSOLUTE_POSITION, ((WaltonSwerveModule)getSwerveModules()[1]).getAzimuthAbsoluteEncoderCounts());
        SmartDashboard.putNumber(DRIVETRAIN_LEFT_REAR_ABSOLUTE_POSITION, ((WaltonSwerveModule)getSwerveModules()[2]).getAzimuthAbsoluteEncoderCounts());
        SmartDashboard.putNumber(DRIVETRAIN_RIGHT_REAR_ABSOLUTE_POSITION, ((WaltonSwerveModule)getSwerveModules()[3]).getAzimuthAbsoluteEncoderCounts());

        SmartDashboard.putNumber(DRIVETRAIN_LEFT_FRONT_RELATIVE_POSITION, ((WaltonSwerveModule)getSwerveModules()[0]).getAzimuthRelativeEncoderCounts());
        SmartDashboard.putNumber(DRIVETRAIN_RIGHT_FRONT_RELATIVE_POSITION, ((WaltonSwerveModule)getSwerveModules()[1]).getAzimuthRelativeEncoderCounts());
        SmartDashboard.putNumber(DRIVETRAIN_LEFT_REAR_RELATIVE_POSITION, ((WaltonSwerveModule)getSwerveModules()[2]).getAzimuthRelativeEncoderCounts());
        SmartDashboard.putNumber(DRIVETRAIN_RIGHT_REAR_RELATIVE_POSITION, ((WaltonSwerveModule)getSwerveModules()[3]).getAzimuthRelativeEncoderCounts());

        SmartDashboard.putNumber(DRIVETRAIN_LEFT_FRONT_ANGLE_DEGREES, ((WaltonSwerveModule)getSwerveModules()[0]).getAzimuthRotation2d().getDegrees());
        SmartDashboard.putNumber(DRIVETRAIN_RIGHT_FRONT_ANGLE_DEGREES, ((WaltonSwerveModule)getSwerveModules()[1]).getAzimuthRotation2d().getDegrees());
        SmartDashboard.putNumber(DRIVETRAIN_LEFT_REAR_ANGLE_DEGREES, ((WaltonSwerveModule)getSwerveModules()[2]).getAzimuthRotation2d().getDegrees());
        SmartDashboard.putNumber(DRIVETRAIN_RIGHT_REAR_ANGLE_DEGREES, ((WaltonSwerveModule)getSwerveModules()[3]).getAzimuthRotation2d().getDegrees());

        SmartDashboard.putNumber(DRIVETRAIN_LEFT_FRONT_VELOCITY_ERROR, ((WaltonSwerveModule)getSwerveModules()[0]).getDriveVelocityError());
        SmartDashboard.putNumber(DRIVETRAIN_RIGHT_FRONT_VELOCITY_ERROR, ((WaltonSwerveModule)getSwerveModules()[1]).getDriveVelocityError());
        SmartDashboard.putNumber(DRIVETRAIN_LEFT_REAR_VELOCITY_ERROR, ((WaltonSwerveModule)getSwerveModules()[2]).getDriveVelocityError());
        SmartDashboard.putNumber(DRIVETRAIN_RIGHT_REAR_VELOCITY_ERROR, ((WaltonSwerveModule)getSwerveModules()[3]).getDriveVelocityError());

        SmartDashboard.putNumber(DRIVETRAIN_SPEED_MSEC, Math.abs(getSwerveModules()[0].getState().speedMetersPerSecond));
    }

    /**
     * Drive the robot with given x, y, and rotational velocities with open-loop velocity control.
     */
    public void drive(
            double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        swerveDrive.drive(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, true);
    }

    /**
     * Move the robot with given x, y, and rotational velocities with closed-loop velocity control.
     */
    public void move(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            boolean isFieldOriented) {
        swerveDrive.move(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, isFieldOriented);
    }

    public void resetHeading() {
        swerveDrive.resetGyro();
    }

    public void setHeadingOffset(Rotation2d offsetRads) {
        swerveDrive.setGyroOffset(offsetRads);
    }

    public Rotation2d getHeading() {
        return swerveDrive.getHeading();
    }

    public PIDController getXController() {
        return xController;
    }

    public PIDController getYController() {
        return yController;
    }

    public ProfiledPIDController getThetaController() {
        return thetaController;
    }

    public void xLockSwerveDrive() {
        ((WaltonSwerveModule) swerveDrive.getSwerveModules()[0])
                .setAzimuthRotation2d(Rotation2d.fromDegrees(45));
        ((WaltonSwerveModule) swerveDrive.getSwerveModules()[1])
                .setAzimuthRotation2d(Rotation2d.fromDegrees(-45));
        ((WaltonSwerveModule) swerveDrive.getSwerveModules()[2])
                .setAzimuthRotation2d(Rotation2d.fromDegrees(-45));
        ((WaltonSwerveModule) swerveDrive.getSwerveModules()[3])
                .setAzimuthRotation2d(Rotation2d.fromDegrees(45));
    }

}