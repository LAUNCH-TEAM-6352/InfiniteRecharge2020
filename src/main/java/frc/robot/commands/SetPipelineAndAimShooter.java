/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Turret;
import frc.util.LimelightCamera;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetPipelineAndAimShooter extends SequentialCommandGroup {
  /**
   * Creates a new SetPipelineAndAimShooter.
   */
  public SetPipelineAndAimShooter(Turret turret) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
		new ConditionalCommand(
			new SequentialCommandGroup(
				new InstantCommand(() ->
				{
					LimelightCamera.getInstance().setPipeline(LimelightConstants.targetingPipelines[LimelightCamera.getInstance().getPipeline()]);
				}),
				new WaitCommand(2.0)
			),
			new WaitCommand(0),
			() -> LimelightConstants.targetingPipelines[LimelightCamera.getInstance().getPipeline()] >= 0
		),
		new AimShooterUsingLimelight(turret)	
	);
  }
}
