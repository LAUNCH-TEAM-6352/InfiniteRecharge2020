/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Add your docs here.
 */
public class Intake extends SubsystemBase
{
	private DoubleSolenoid solenoid = new DoubleSolenoid(IntakeConstants.outSolenoidChannel, IntakeConstants.inSolenoidChannel);

	private VictorSPX motor = new VictorSPX(IntakeConstants.motorChannel);

	public void moveIntakeIn()
	{
		solenoid.set(DoubleSolenoid.Value.kReverse);
	}

	public void moveIntakeOut()
	{
		solenoid.set(DoubleSolenoid.Value.kForward);
	}
}
