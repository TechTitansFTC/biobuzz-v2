package org.firstinspires.ftc.teamcode.util.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class PathChainBuilder {
    public static Path buildPathChain(double[] startPosition, double[] endPosition) {
        Pose startPose = new Pose(startPosition[0], startPosition[1], Math.toRadians(startPosition[2]));
        Pose endPose = new Pose(endPosition[0], endPosition[1], Math.toRadians(endPosition[2]));

        return Paths.line(startPose, endPose).linear(startPose, endPose);
    }
    public static Path buildPathChain(double[] startPosition, double[] endPosition, double velocityConstraint, double timeoutMs) {
        Pose startPose = new Pose(startPosition[0], startPosition[1], Math.toRadians(startPosition[2]));
        Pose endPose = new Pose(endPosition[0], endPosition[1], Math.toRadians(endPosition[2]));

        return Paths.line(startPose, endPose)
                .linear(startPose, endPose)
                .with(
                        Constants.foresightConfig.maxPathSpeed.at(velocityConstraint),
                        Constants.foresightConfig.timeoutConstraint.at(timeoutMs)
                );
    }

}