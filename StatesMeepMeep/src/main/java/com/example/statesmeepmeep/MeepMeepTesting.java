package com.example.statesmeepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //50, 30
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(60), 11)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(270)))

                                .lineToSplineHeading(new Pose2d(-55, -40, Math.toRadians(360)))
                                .lineToConstantHeading(new Vector2d(-55, -20))

                                //drop freight
                                .lineToConstantHeading(new Vector2d(-33, -25))
                                .UNSTABLE_addTemporalMarkerOffset(-2.5, () -> {
                                    /*rightSlide.setPower(.5);
                                    leftSlide.setPower(.5);
                                    rightSlide.setTargetPosition((int) cameraslideValuesred[barcodeValue]);
                                    leftSlide.setTargetPosition((int) cameraslideValuesred[barcodeValue]);
                                    rightDropdown.setPosition(forward);
                                    leftDropdown.setPosition(forward);
                                    stick.setPosition(stickDown);*/})
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    /*stick.setPosition(stickUp);*/})
                                .waitSeconds(.3)

                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    /*stick.setPosition(stickDown);*/})
                                .UNSTABLE_addTemporalMarkerOffset(.4, () -> {
                                    /*stick.setPosition(stickDown);
                                    rightSlide.setVelocity(maxSlideVelocity * .5);
                                    leftSlide.setVelocity(maxSlideVelocity * .5);
                                    leftDropdown.setPosition(down + .05);
                                    rightDropdown.setPosition(down + .05);
                                    rightSlide.setTargetPosition(0);
                                    leftSlide.setTargetPosition(0)
                                    ;*/})
                                .lineToSplineHeading(new Pose2d(-62, -30, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-60, -60, Math.toRadians(90)))
                                .UNSTABLE_addTemporalMarkerOffset(-.5, () -> {
                                    /*carousel.setPower(-.4);*/})
                                .waitSeconds(3)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    /*carousel.setPower(0);*/})

                                .lineToConstantHeading(new Vector2d(-59, -66))
                                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                                    /*tubeys.setPower(.82);
                                    stick.setPosition(stickUp);
                                    leftDropdown.setPosition(down);
                                    rightDropdown.setPosition(down);*/})

                                .lineToConstantHeading(new Vector2d(-40, -62))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    /*stick.setPosition(stickDown);*/})
                                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                                    /*tubeys.setPower(0);*/})
                                .lineToSplineHeading(new Pose2d(-55, -40, Math.toRadians(360)))
                                .lineToConstantHeading(new Vector2d(-55, -20))
                                //drop duck
                                .lineToConstantHeading(new Vector2d(-33, -25))
                                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                                    /*rightSlide.setPower(.5);
                                    leftSlide.setPower(.5);
                                    rightSlide.setTargetPosition((int) maxSlideTicks);
                                    leftSlide.setTargetPosition((int) maxSlideTicks);
                                    rightDropdown.setPosition(forward);
                                    leftDropdown.setPosition(forward);
                                    stick.setPosition(stickDown);*/})
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    /*stick.setPosition(stickUp);*/})
                                .waitSeconds(.3)

                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    /*stick.setPosition(stickDown);*/})
                                .UNSTABLE_addTemporalMarkerOffset(.4, () -> {
                                    /*stick.setPosition(stickDown);
                                    rightSlide.setVelocity(maxSlideVelocity * .5);
                                    leftSlide.setVelocity(maxSlideVelocity * .5);
                                    leftDropdown.setPosition(down + .05);
                                    rightDropdown.setPosition(down + .05);
                                    rightSlide.setTargetPosition(0);
                                    leftSlide.setTargetPosition(0);*/})
                                .lineToSplineHeading(new Pose2d(-62, -30, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-64, -40, Math.toRadians(90)))
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}