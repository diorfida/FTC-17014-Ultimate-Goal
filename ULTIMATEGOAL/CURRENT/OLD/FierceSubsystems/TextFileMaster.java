package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.OLD.FierceSubsystems.Drivetrain.DrivetrianHardware;

import java.io.File;

/**
 * Class that houses all of the methods for controlling the text files.
 * @author Domenic Iorfida, FTC 17014
 */
public class TextFileMaster {

    private DrivetrianHardware drivetrain;

    /**
     * Constructor.
     * @param drivetrain Instance of the drivetrain hardware.
     */
    public TextFileMaster(DrivetrianHardware drivetrain){
        this.drivetrain = drivetrain;
    }

    /**
     * Checks to see if the text files have any data in them.
     * @param x File for the x coordinate.
     * @param y File for the y coordinate
     * @param heading File for the heading.
     */
    public void checkTextFiles(File x, File y, File heading){
        if (x.exists() && y.exists() && heading.exists()){
            x.delete();
            y.delete();
            heading.delete();
        }
    }

    /**
     * Updates the text files with the new information.
     * It is intended to be used in the stop method of an autonomous op-mode.
     * @param x File for the x coordinate.
     * @param y File for the y coordinate.
     * @param heading File for the heading.
     */
    public void updateTextFiles(File x, File y, File heading){
        drivetrain.updatePoseEstimate();
        Pose2d finalPose = drivetrain.getPoseEstimate();

        ReadWriteFile.writeFile(x, String.valueOf(finalPose.getX()));
        ReadWriteFile.writeFile(y, String.valueOf(finalPose.getY()));
        ReadWriteFile.writeFile(heading, String.valueOf(finalPose.getHeading()));
    }

    /**
     * Reads the text files and sets the robot 2D pose information.
     * @param x File for the X coordinate.
     * @param y File for the Y coordinate.
     * @param heading File for the heading.
     * @return If there is any data in the files and the Pose2D has successfully been set.
     */
    public boolean DeriveTextFiles(File x, File y, File heading){
        if (x.exists() && y.exists() && heading.exists()){
            double startingX = Double.parseDouble(ReadWriteFile.readFile(x).trim());
            double startingY = Double.parseDouble(ReadWriteFile.readFile(y).trim());
            double startingHeading = Double.parseDouble(ReadWriteFile.readFile(heading).trim());

            if (startingX == 0 && startingY == 0 && startingHeading == 0){
                return false;
            }else {
                Pose2d startingPose = new Pose2d(startingX, startingY, startingHeading);
                drivetrain.setPoseEstimate(startingPose);
                return true;
            }
        }else {
            return false;
        }
    }
}