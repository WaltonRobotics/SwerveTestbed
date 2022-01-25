package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

import static frc.robot.OI.*;
import static frc.robot.Robot.intake;

public class IntakeCommand extends CommandBase {

    public IntakeCommand(){
        addRequirements(intake);
    }

    @Override
    public void execute(){
        if(intakeButton.get()){
            intake.setRollerDutyCycle(0.8);
        }
    }

    @Override
    public void end(boolean interrupted){
        intake.setRollerDutyCycle(0);
    }

}
