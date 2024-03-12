package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoDriveCommand extends Command{
    private double x, y;
    private SwerveDriveSubsystem drive;
    private Command command;
    Timer timer;

    /**Goes wherever you tell it!
     * @param swerveDriveSubsystem - to drive obviously
     * @param x - however much you want to move on the x axis
     * @param y - however much you want to move on the y axis
     */
    public AutoDriveCommand(SwerveDriveSubsystem swerveDriveSubsystem, double x, double y) {
        this.x = x;
        this.y = y;

        drive = swerveDriveSubsystem;
        timer = new Timer();

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        timer.start();
        // Get our current pose
        Pose2d currentPose = drive.getPose();

        Pose2d startPose = new Pose2d(currentPose.getTranslation(), new Rotation2d());
        Pose2d endPose = new Pose2d(currentPose.getTranslation().plus(new Translation2d(x, y)), new Rotation2d());

        // Create a list of points 
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPose, endPose);
        
        // Create the autonomous path
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints, 
            new PathConstraints(
                4.0, 
                4.0,
                Units.degreesToRadians(360), 
                Units.degreesToRadians(540)
                ), 
            // Our desired end state is not moving and facing the same
            new GoalEndState(0.0, new Rotation2d())
        );

        // Don't flip the path
        path.preventFlipping = true;
        
        // Schedule the command to follow the path
        
        AutoBuilder.followPath(path).schedule();
        command = new WaitCommand(2);
        command.schedule();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
}
