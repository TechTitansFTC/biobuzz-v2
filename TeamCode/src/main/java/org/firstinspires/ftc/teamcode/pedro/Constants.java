package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    // pinpoint: port 3 i2c control hub

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeftDrive");
        c.frontRightName.set("frontRightDrive");
        c.backLeftName.set("backLeftDrive");
        c.backRightName.set("backRightDrive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-4.113466157687931);
        c.yPodOffset.set(-7.664149727408342);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3917570388600592);
                Controller secondaryTranslationalForward = Controller.proportional(0.14474375791792268);
                Controller primaryTranslationalLateral = Controller.proportional(0.359938293458261);
                Controller secondaryTranslationalLateral = Controller.proportional(0.26230322953225255);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.015627013215231692));
                c.brake.set(Controller.proportionalFeedforward(0.013282961232946937));

                c.headingFeedback.set(Controller.proportional(.5556314483103265));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05690967205533455, 0.0051598120941477935));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09233193199568193, 0.08335173192952278));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012088314109617484, 0.001098251201321267));

                c.maxAchievableForwardVelocity.set(67.56695810367616);
                c.maxAchievableStrafeVelocity.set(51.20090854185859);
                c.naturalForwardDeceleration.set(60.74130085856973);
                c.naturalStrafeDeceleration.set(90.31880106791107);
            }
    );
}