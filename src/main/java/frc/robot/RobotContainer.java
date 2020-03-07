/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.beans.IndexedPropertyChangeEvent;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AimShooterUsingLimelight;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.MoveHoodToDownPosition;
import frc.robot.commands.MoveHoodToPosition;
import frc.robot.commands.MoveHoodToUpPosition;
import frc.robot.commands.MoveTurretToCenterPosition;
import frc.robot.commands.RunTurretWithGameController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.util.LimelightCamera;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
	// Subsystems:
	private Shooter shooter;
	private Turret turret;
	private DriveTrain driveTrain;
	private Intake intake;
	private Indexer indexer;

	// OI devices:
	private final XboxController xboxController;
	private final Joystick leftStick;
	private final Joystick rightStick;

	// Camera:
	private final LimelightCamera limelightCamera;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer()
	{
		// Create OI devices:
		xboxController = new XboxController(OIConstants.xboxControllerPort);
		leftStick = new Joystick(OIConstants.leftJoystickPort);
		rightStick = new Joystick(OIConstants.rightJoystickPort);

		// Create subsystems:
		driveTrain = new DriveTrain();
		shooter = new Shooter(xboxController);
		turret = new Turret(xboxController);
		intake = new Intake();
		indexer = new Indexer();

		// Create camera:
		limelightCamera = LimelightCamera.getInstance();

		// Configure default commands:
		driveTrain.setDefaultCommand(new DriveWithJoysticks(driveTrain, leftStick, rightStick));

		turret.setDefaultCommand(new RunTurretWithGameController(turret, xboxController));

		shooter.setDefaultCommand(new RunCommand(
			() -> shooter.setPercentage(xboxController.getTriggerAxis(Hand.kLeft)),
			shooter
		));

		// Configure the button bindings
		configureButtonBindings();

		initSmartDashboard();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings()
	{
		new JoystickButton(xboxController, Button.kStart.value)
			.whenPressed(new AimShooterUsingLimelight(turret));
		
		new JoystickButton(xboxController, Button.kStickLeft.value)
			.whenPressed(new MoveTurretToCenterPosition(turret));

		new JoystickButton(xboxController, Button.kY.value)
			.whenPressed(new MoveHoodToUpPosition(turret));

		new JoystickButton(xboxController, Button.kA.value)
			.whenPressed(new MoveHoodToDownPosition(turret));
		
		new JoystickButton(xboxController, Button.kBumperLeft.value)
			.whenPressed(new InstantCommand(() -> intake.moveIntakeOut()));
		
		new JoystickButton(xboxController, Button.kBumperRight.value)
			.whenPressed(new InstantCommand(() -> intake.moveIntakeIn()));
	}

	private void initSmartDashboard()
	{
		SmartDashboard.putNumber(DashboardConstants.shooterTargetVelocityKey, DashboardConstants.shooterTargetVelocityDefault);
		SmartDashboard.putNumber(DashboardConstants.hoodTargetPositionKey, DashboardConstants.hoodTargetPositionDefault);
		SmartDashboard.putNumber(DashboardConstants.indexerPercentageKey, DashboardConstants.indexerPercentageDefault);

		SmartDashboard.putData("LL: Driver1", new InstantCommand(() -> limelightCamera.setPipeline(LimelightConstants.pipelineDriver1)));
		SmartDashboard.putData("LL: Driver2", new InstantCommand(() -> limelightCamera.setPipeline(LimelightConstants.pipelineDriver2)));
		SmartDashboard.putData("LL: Driver3", new InstantCommand(() -> limelightCamera.setPipeline(LimelightConstants.pipelineDriver3)));
		SmartDashboard.putData("LL: Vision1", new InstantCommand(() -> limelightCamera.setPipeline(LimelightConstants.pipelineVision1)));
		SmartDashboard.putData("LL: Vision2", new InstantCommand(() -> limelightCamera.setPipeline(LimelightConstants.pipelineVision2)));
		SmartDashboard.putData("LL: Vision3", new InstantCommand(() -> limelightCamera.setPipeline(LimelightConstants.pipelineVision3)));

		//SmartDashboard.putData("Target Shooter", new SetPipelineAndAimShooter(turret));

		SmartDashboard.putData("Move Hood", new MoveHoodToPosition(turret, DashboardConstants.hoodTargetPositionKey));

		SmartDashboard.putData("Run Shooter Vel", new StartEndCommand(
			() -> shooter.setVelocity(
				SmartDashboard.getNumber(DashboardConstants.shooterTargetVelocityKey, 0)),
			() -> shooter.setPercentage(0),
			shooter
			)
		);

		SmartDashboard.putData("Run Shooter %", new StartEndCommand(
			() -> shooter.setPercentage(
				SmartDashboard.getNumber(DashboardConstants.shooterTargetPercentageKey, 0)),
			() -> shooter.setPercentage(0),
			shooter
			)
		);

		SmartDashboard.putData("Run Indexer %", new StartEndCommand(
			() -> indexer.set(
				SmartDashboard.getNumber(DashboardConstants.indexerPercentageKey, 0)),
			() -> indexer.stop(),
			indexer
			)
		);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand()
	{
		// An ExampleCommand will run in autonomous
		return null;
	}
}
