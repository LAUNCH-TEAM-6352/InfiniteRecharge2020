/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.AimShooterUsingLimelight;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.ExtendClimberActuators;
import frc.robot.commands.MoveHoodToPosition;
import frc.robot.commands.MoveTurretToCenterPosition;
import frc.robot.commands.RunClimberWithGameController;
import frc.robot.commands.RunIndexer;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunTurretWithGameController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.util.LimelightCamera;


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
	private final Shooter shooter;
	private final Turret turret;
	private final DriveTrain driveTrain;
	private final Intake intake;
	private final Indexer indexer;
	private final Climber climber;

	// OI devices:
	private final XboxController gameController;
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
		gameController = new XboxController(OIConstants.xboxControllerPort);
		leftStick = new Joystick(OIConstants.leftJoystickPort);
		rightStick = new Joystick(OIConstants.rightJoystickPort);

		// Create subsystems:
		driveTrain = new DriveTrain();
		shooter = new Shooter(gameController);
		turret = new Turret(gameController);
		intake = new Intake();
		indexer = new Indexer();
		climber = new Climber(gameController);

		// Create camera:
		limelightCamera = LimelightCamera.getInstance();

		// Configure default commands:
		driveTrain.setDefaultCommand(new DriveWithJoysticks(driveTrain, leftStick, rightStick));
		turret.setDefaultCommand(new RunTurretWithGameController(turret, gameController));
		climber.setDefaultCommand(new RunClimberWithGameController(climber, gameController));

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
		// Start shooter motors, switch to vision procesing, aim turret
		new JoystickButton(gameController, Button.kA.value)
			.whenPressed(new ParallelCommandGroup(
				new RunShooter(shooter, DashboardConstants.shooterTargetVelocityKey),
				new SequentialCommandGroup(
					new ConditionalCommand(
						new SequentialCommandGroup(
							new InstantCommand(() ->
								LimelightCamera.getInstance().setPipeline(LimelightConstants.targetingPipelines[LimelightCamera.getInstance().getPipeline()])
							),
							new WaitCommand(2.0)
						),
						new WaitCommand(0),
						() -> LimelightConstants.targetingPipelines[LimelightCamera.getInstance().getPipeline()] >= 0
					),
					new AimShooterUsingLimelight(turret)
				)		
			));

		// Stop the shooter motors and restore the Limelight to driver mode:
		new JoystickButton(gameController, Button.kY.value)
			.whenPressed(new ParallelCommandGroup(
				new InstantCommand(() -> shooter.stop(), shooter),
				new ConditionalCommand(
					new InstantCommand(() ->
						LimelightCamera.getInstance().setPipeline(LimelightConstants.drivingPipelines[LimelightCamera.getInstance().getPipeline()])
					),
					new WaitCommand(0),
					() -> LimelightConstants.drivingPipelines[LimelightCamera.getInstance().getPipeline()] >= 0
				)	
			));

		// Run the indexer forward:
		new JoystickButton(gameController, Button.kX.value)
			.whileHeld(new RunIndexer(indexer, DashboardConstants.indexerForwardPercentageKey));

		// Run the indexer reverse:
		new JoystickButton(gameController, Button.kB.value)
			.whileHeld(new RunIndexer(indexer, DashboardConstants.indexerReversePercentageKey));

		// Move the turret to the center position:
		new JoystickButton(gameController, Button.kStickLeft.value)
			.whenPressed(new MoveTurretToCenterPosition(turret));

		// Set the limelight to 1x driver mode:
		new JoystickButton(gameController, Button.kBumperLeft.value)
			.whenPressed(new InstantCommand(() -> limelightCamera.setPipeline(LimelightConstants.pipelineDriver1)));

		// Set the limelight to 2x driver mode:
		new JoystickButton(gameController, Button.kBumperRight.value)
			.whenPressed(new InstantCommand(() -> limelightCamera.setPipeline(LimelightConstants.pipelineDriver2)));

		// Extend the climber actuators:
		new JoystickButton(gameController, Button.kStart.value)
			.whenPressed(new ExtendClimberActuators(climber));

		// Button 1 is the trigger
		// Run the intake in the in direction:
		new JoystickButton(leftStick, 1)
			.whileHeld(new RunIntake(intake, DashboardConstants.intakeInPercentageKey));

		// RUn the intake in the out direction:
		new JoystickButton(rightStick, 1)
			.whileHeld(new RunIntake(intake, DashboardConstants.intakeOutPercentageKey));

		// Move the intake out:
		new JoystickButton(leftStick, 6)
			.whenPressed(new InstantCommand(() -> intake.moveIntakeOut()));
		
		// Move the intake in:
		new JoystickButton(rightStick, 5)
			.whenPressed(new InstantCommand(() -> intake.moveIntakeIn()));
		
		// Deploy the climber linear actuators:
		new JoystickButton(leftStick, 11)
			.whenPressed(new InstantCommand(() -> climber.deployActuators()));		
	}

	private void initSmartDashboard()
	{
		SmartDashboard.putNumber(DashboardConstants.shooterTargetVelocityKey, ShooterConstants.defaultVelocity);
		SmartDashboard.putNumber(DashboardConstants.hoodTargetPositionKey, TurretConstants.defaultHoodTargetPosition);
		SmartDashboard.putNumber(DashboardConstants.indexerForwardPercentageKey, IndexerConstants.defaultForwardSpeed);
		SmartDashboard.putNumber(DashboardConstants.indexerReversePercentageKey, IndexerConstants.defaultReverseSpeed);
		SmartDashboard.putNumber(DashboardConstants.intakeInPercentageKey, IntakeConstants.defaultInSpeed);
		SmartDashboard.putNumber(DashboardConstants.intakeOutPercentageKey, IntakeConstants.defaultOutSpeed);

		SmartDashboard.putData("LL: Driver1", new InstantCommand(() -> limelightCamera.setPipeline(LimelightConstants.pipelineDriver1)));
		SmartDashboard.putData("LL: Driver2", new InstantCommand(() -> limelightCamera.setPipeline(LimelightConstants.pipelineDriver2)));
		SmartDashboard.putData("LL: Driver3", new InstantCommand(() -> limelightCamera.setPipeline(LimelightConstants.pipelineDriver3)));
		SmartDashboard.putData("LL: Vision1", new InstantCommand(() -> limelightCamera.setPipeline(LimelightConstants.pipelineVision1)));
		SmartDashboard.putData("LL: Vision2", new InstantCommand(() -> limelightCamera.setPipeline(LimelightConstants.pipelineVision2)));
		SmartDashboard.putData("LL: Vision3", new InstantCommand(() -> limelightCamera.setPipeline(LimelightConstants.pipelineVision3)));

		SmartDashboard.putData("Undeploy Climber Actuators", new InstantCommand(() -> climber.undeployActuators()));

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
				SmartDashboard.getNumber(DashboardConstants.indexerForwardPercentageKey, 0)),
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
