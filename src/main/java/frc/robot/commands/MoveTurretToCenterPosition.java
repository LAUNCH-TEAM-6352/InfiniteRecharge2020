/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

/**
 * Moves the turret to the center position.
 */
public class MoveTurretToCenterPosition extends CommandBase
{
	@SuppressWarnings(
	{ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Turret turret;

	public MoveTurretToCenterPosition(Turret turret)
	{
		this.turret = turret;

		// Use addRequirements() here to declare subsystem dependencies.
		if (turret != null)
		{
			addRequirements(turret);
		}
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize()
	{
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute()
	{
		turret.moveToCenterPosition();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted)
	{
		turret.setAzimuth(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished()
	{
		return turret.isAtCenterPosition();
	}
}
