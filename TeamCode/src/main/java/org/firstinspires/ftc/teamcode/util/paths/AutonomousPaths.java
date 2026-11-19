package org.firstinspires.ftc.teamcode.util.paths;

import java.util.Arrays;

public class AutonomousPaths {
    private double[] initial;
    private double[] down;
    private double[] middle;
    private double[] center;
    private double[] up;
    private double[] flowerUp1;
    private double[] flowerUp2;
    private double[] flowerDown1;
    private double[] flowerDown2;
    private double[] garden1;
    private double[] garden2;
    private double[] park;
    public AutonomousPaths() {
        initial = new double[]{56, 9, 0}; // x, y, heading
        down = new double[]{58, 18, 0};
        middle = new double[]{9, 93, 90};
        center = new double[]{80, 72, 0};
        up = new double[]{58, 126, 180};
        flowerUp1 = new double[]{48, 123, 90};
        flowerUp2 = new double[]{48, 133, 90};
        flowerDown1 = new double[]{20, 48, 180};
        flowerDown2 = new double[]{12.5, 48, 180};
        garden1 = new double[]{10, 17, 270};
        garden2 = new double[]{10, 10, 270};
        park = new double[]{9, 95, 90};
    }

    public double[] getInitial() {
        return initial;
    }

    public double[] getDown() {
        return down;
    }

    public double[] getMiddle() {
        return middle;
    }
    public double[] getCenter() { return center; }

    public double[] getUp() {
        return up;
    }

    public double[] getFlowerUp1() {
        return flowerUp1;
    }

    public double[] getFlowerUp2() {
        return flowerUp2;
    }

    public double[] getFlowerDown1() {
        return flowerDown1;
    }

    public double[] getFlowerDown2() {
        return flowerDown2;
    }

    public double[] getGarden1() {
        return garden1;
    }

    public double[] getGarden2() {
        return garden2;
    }

    public double[] getPark() {
        return park;
    }

    public String getPose(double[] pose) {
        if (Arrays.equals(pose, initial)) return "initial";
        if (Arrays.equals(pose, down)) return "down";
        if (Arrays.equals(pose, middle)) return "middle";
        if (Arrays.equals(pose, up)) return "up";
        if (Arrays.equals(pose, flowerUp1)) return "flowerUp1";
        if (Arrays.equals(pose, flowerUp2)) return "flowerUp2";
        if (Arrays.equals(pose, flowerDown1)) return "flowerDown1";
        if (Arrays.equals(pose, flowerDown2)) return "flowerDown2";
        if (Arrays.equals(pose, garden1)) return "garden1";
        if (Arrays.equals(pose, garden2)) return "garden2";
        if (Arrays.equals(pose, park)) return "park";

        return "unknown";
    }

    // pass through zz on:
    /*
     * up to down
     * down to flowerUp
     * up to flowerDown
     * up to garden
     */
}