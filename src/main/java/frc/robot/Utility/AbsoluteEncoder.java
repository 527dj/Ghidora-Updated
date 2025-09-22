package frc.robot.Utility;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AbsoluteEncoder {

    private DutyCycleEncoder encoder;

    private int prevQuadrant = 1;

    private int numRotations = 0;

    private Rotation2d offset = new Rotation2d();

    private boolean inverted = false;

    public AbsoluteEncoder(int port){
        encoder = new DutyCycleEncoder(new DigitalInput(port));

        //init quadrant
        Rotation2d currentReading = getRawPosition();

        if(currentReading.getDegrees() >= 0 && currentReading.getDegrees() < 90)
            prevQuadrant = 1;
        else if(currentReading.getDegrees() >= 90 && currentReading.getDegrees() < 180)
            prevQuadrant = 2;
        else if(currentReading.getDegrees() >= 180 && currentReading.getDegrees() < 270)
            prevQuadrant = 3;
        else if(currentReading.getDegrees() >= 270 && currentReading.getDegrees() < 360)
            prevQuadrant = 4;
    }   

    public DutyCycleEncoder getEncoder(){
        return encoder;
    }

    public Rotation2d getPosition(){
        Rotation2d currentReading = getRawPosition();
       
        int quadrant;

        if(currentReading.getDegrees() >= 0 && currentReading.getDegrees() < 90)
            quadrant = 1;
        else if(currentReading.getDegrees() >= 90 && currentReading.getDegrees() < 180)
            quadrant = 2;
        else if(currentReading.getDegrees() >= 180 && currentReading.getDegrees() < 270)
            quadrant = 3;
        else if(currentReading.getDegrees() >= 270 && currentReading.getDegrees() < 360)
            quadrant = 4;
        else
            quadrant = 1;

        if(quadrant != prevQuadrant){

            if(quadrant == 1 && prevQuadrant == 4)
                numRotations++;
            else if(quadrant == 4 && prevQuadrant == 1)
                numRotations--;

            prevQuadrant = quadrant;
        }

        Rotation2d wrappedAngle = inverted ? 
            Rotation2d.fromRotations(currentReading.getRotations() + numRotations).times(-1) : 
            Rotation2d.fromRotations(currentReading.getRotations() + numRotations);

        return Rotation2d.fromRotations(wrappedAngle.getRotations() + offset.getRotations());
    }

    public Rotation2d getRawPosition(){
        Rotation2d currentReading = Rotation2d.fromRotations(encoder.get());

        return currentReading;
    }

    public void setWraps(int newWraps){
        numRotations = newWraps;
    }

    public int getWraps(){
        return numRotations;
    }

    public void setOffset(Rotation2d offset){
        this.offset = offset;
    }

    public Rotation2d getOffset(){
        return offset;
    }

    public void setInverted(boolean invert){
        inverted = invert;
    }

    public boolean isInverted(){
        return inverted;
    }
}
