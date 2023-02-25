// package frc.robot.commands.teleop;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// import static frc.robot.OI.intakeButton;
// import static frc.robot.OI.outtakeButton;
// import static frc.robot.Robot.intake;

// public class IntakeCommand extends CommandBase {

//     public IntakeCommand() {
//         addRequirements(intake);


//     }

//     @Override
//     public void execute() {
//         if (intakeButton.get()) {


//             intake.setVoltage(SmartDashboard.getNumber("Intake voltage", 8));
//         } else if (outtakeButton.get()) {
//             intake.setVoltage(-SmartDashboard.getNumber("Intake voltage", 8));
//         } else {
//             intake.setVoltage(0.0);
//         }

// //        intake.setVoltage(8.0);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         intake.setVoltage(0);
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
