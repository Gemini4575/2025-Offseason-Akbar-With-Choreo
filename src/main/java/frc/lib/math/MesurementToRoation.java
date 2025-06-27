package frc.lib.math;

import edu.wpi.first.math.util.Units;

public class MesurementToRoation {

    private double calculateRotations(double inches, double gearboxRatio, double shaftCircumference) {
        double shaftCircumference2 = Math.PI * shaftCircumference; // Circumference of the 1/2 inch shaft
        double rotationsForInches = inches / shaftCircumference2; // Rotations needed for given inches
        double rotationsWithGearbox = rotationsForInches * gearboxRatio; // Adjust for gearbox
        return rotationsWithGearbox;
    }

    /**
     * Calculate the ticks needed to move a certain distance
     * 
     * @param inches       amount of inches to move
     * @param gearboxRatio the ratio of the gearbox on the motor
     * @return encoder ticks needed to move the distance
     */
    private double calculateTicks(double inches, double gearboxRatio, double shaftCircumference) {
        double rotations = calculateRotations(inches, gearboxRatio, shaftCircumference);
    //  double ticksPerRotation = 42.0; // Ticks per one rotation per rev website not Mr.Fran
        return rotations;
    }

    public double calculateRotationsIN(double inches, double gearboxRatio, double shaftCircumference) {
        return calculateTicks(inches, gearboxRatio, shaftCircumference);
    }

    public double calculateRotationsM(double meters, double gearboxRatio, double shaftCircumference) {
        return calculateTicks(Units.metersToInches(meters), gearboxRatio, shaftCircumference);
    }

    public double calculateRotationsCM(double centemeters, double gearboxRatio, double shaftCircumference) {
        return calculateTicks(Units.metersToInches(centemeters/100), gearboxRatio, shaftCircumference);
    }

    public double calculateRotationsFT(double feet, double gearboxRatio, double shaftCircumference) {
        return calculateTicks(feet * 12, gearboxRatio, shaftCircumference);
    }

}
