package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DebuggingLog;

import java.util.logging.Level;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Intake extends SubsystemBase{
    public boolean kIsIntakeControllerInverted;
    public double kIntakeDutyCycle;
    public double kOuttakeDutyCycle;
    public double kSettleTime;

    private final Spark mIntakeController = new Spark(9);//kIntakeID dummy value
    //private final Solenoid mDeploySolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1); //deployIntakeSolenoid ID

    public Intake(){
        mIntakeController.setInverted(true);
    }

//    public void setDeployed(boolean isDeployed) {
//        mDeploySolenoid.set(isDeployed);
//    }

    public void setVoltage(double voltage) {
        mIntakeController.setVoltage(voltage);
    }
}