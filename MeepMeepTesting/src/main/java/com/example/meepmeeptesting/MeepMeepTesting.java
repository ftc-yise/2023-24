package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity redLeft = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.38)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-41.5, -61, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-42, -43, Math.toRadians(-60)))
                                .lineToLinearHeading(new Pose2d(-32, -43, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-32, -12, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(42, -30, Math.toRadians(180)))
                                .back(10)
                                .build()
                );

        RoadRunnerBotEntity redCenter = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.38)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-45, 61, Math.toRadians(90)))
                                .waitSeconds(6)
                                .lineToLinearHeading(new Pose2d(-47, 43, Math.toRadians(90)))
                                .strafeLeft(10)
                                .back(30)
                                .lineToLinearHeading(new Pose2d(-20, 12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(35, 12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(45, 42, Math.toRadians(180)))
                                .forward(-5)
                                .forward(15)
                                .lineToLinearHeading(new Pose2d(60, 60, Math.toRadians(180)))
                                .build()
                );

        // Declare our first bot
        RoadRunnerBotEntity BlueInnerBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.38)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(17, 61, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(22, 33, Math.toRadians(0)))
                                .back(9)
                                .back(-6)
                                .lineToLinearHeading(new Pose2d(38, 68, Math.toRadians(180)))
                                .strafeLeft(4)
                                .lineToLinearHeading(new Pose2d(39, 34, Math.toRadians(180)))
                                .forward(-5)
                                .forward(10)
                                .strafeLeft(28)
                                .forward(-22)
                                .build()
                );

        RoadRunnerBotEntity redRight = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.38)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-41.5, -61, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-32, -40, Math.toRadians(-130)))
                                .lineToLinearHeading(new Pose2d(-42, -43, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-42, -12, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(42, -38, Math.toRadians(180)))
                                .back(10)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(redRight)
                .addEntity(redCenter)
                .addEntity(redLeft)
                .start();
    }
}