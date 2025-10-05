package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;

public class PlayMusicCommand extends Command {
    private final Orchestra orchestra;
    private boolean hasPlayed = false;

    /**
     * Creates a command to play music on TalonFX motors at robot startup.
     * This command does not require any subsystems, so it won't interfere with other commands.
     * 
     * @param motors The TalonFX motors to use for playing music
     */
    public PlayMusicCommand(TalonFX... motors) {
        orchestra = new Orchestra();
        
        // Add all TalonFX motors to the orchestra
        for (TalonFX motor : motors) {
            orchestra.addInstrument(motor);
        }
        
        // Load your music file from src/main/deploy
        // Create a .chrp file using Phoenix Tuner X and place it in deploy folder
        orchestra.loadMusic("song.chrp");
    }

    @Override
    public void initialize() {
        System.out.println("Starting music playback...");
        orchestra.play();
        hasPlayed = false;
    }

    @Override
    public void execute() {
        // Command stays active while music plays
    }

    @Override
    public void end(boolean interrupted) {
        orchestra.stop();
        if (interrupted) {
            System.out.println("Music playback interrupted");
        } else {
            System.out.println("Music playback completed");
        }
    }

    @Override
    public boolean isFinished() {
        // Finish when music stops playing
        if (orchestra.isPlaying()) {
            hasPlayed = true;
            return false;
        }
        return hasPlayed; // Only finish after music has played and stopped
    }
}