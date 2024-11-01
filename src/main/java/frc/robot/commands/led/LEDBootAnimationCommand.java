package frc.robot.commands.led;

import static edu.wpi.first.wpilibj.util.Color.kBlack;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStrips;
import frc.robot.subsystems.LEDSubsystem;

/** Command to run the boot animation on the LED strips */
public class LEDBootAnimationCommand extends Command {

	private static final int BLIP_SIZE = 5;
	private final LEDSubsystem ledSubsystem;
	private final Timer timer = new Timer();

	private int blipIndex = -1;
	private boolean done = false;
	private boolean initialized = false;

	public LEDBootAnimationCommand(LEDSubsystem ledSubsystem) {
		this.ledSubsystem = ledSubsystem;

		addRequirements(ledSubsystem);
	}

	@Override
	public void initialize() {
		blipIndex = -1;
		timer.start();
		ledSubsystem.setUpdater(this::animate);
		done = false;
		initialized = false;
	}

	private void animate(LEDStrips ledStrips) {
		if (timer.advanceIfElapsed(0.05) || !initialized) {
			if (!initialized) {
				ledStrips.setAll(kBlack);
				initialized = true;
			}
			for (int index = 0; index < ledStrips.getMirrorStripSize(); index++) {
				if (index <= blipIndex && index >= blipIndex - (BLIP_SIZE - 1)) {
					ledStrips.setMirrorLED(index, Color.kOrange);
				} else {
					ledStrips.setMirrorLED(index, Color.kBlue);
				}
			}
			blipIndex++;
			ledStrips.refresh();
			done = blipIndex - (BLIP_SIZE + 1) >= ledStrips.getMirrorStripSize();
		}
	}

	@Override
	public boolean isFinished() {
		return done;
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

	@Override
	public void end(boolean interrupted) {
		ledSubsystem.setUpdater(null);
	}
}
