package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Util.Elastic;
import frc.robot.Util.Elastic.Notification;
import frc.robot.Util.Elastic.Notification.NotificationLevel;

public enum Notifications {
    GENERAL(NotificationLevel.INFO, "General", "%s"),
    SWERVE_HOME_SUCCESS(NotificationLevel.INFO, "Swerve", "Homed all swerve modules"),
    SWERVE_HOME_FAIL(NotificationLevel.ERROR, "Swerve", "Failed to home all swerve modules"),
    PATH_SCHEDULED(NotificationLevel.INFO, "Navigation", "Navigating to (%s, %s) at %s degrees heading"),
    ELEVATOR_HOME_SUCCESS(NotificationLevel.INFO, "Elevator", "Homed elevator"),
    ELEVATOR_HOME_FAIL(NotificationLevel.ERROR, "Elevator", "Failed to home elevator"),
    ELEVATOR_INVALIDE_HEIGHT(NotificationLevel.ERROR, "Elevator", "Elevator commanded to move outside its bounds (%s)")
    ;
    
    private final Notification notification;
    private Notifications(NotificationLevel level, String title, String description) {
        notification = new Notification(level, title, description, 
            (level == NotificationLevel.INFO) ?     3000 :
            (level == NotificationLevel.WARNING) ?  4000 :
                                        /*error*/   5000
        );
    }

    /**Send the notification represented by this enum object to the Elastic dashboard
     * @param formatting The formatting to apply to the string using String.format()
     * @return The command to send the notification to elastic; ends immediately
     */
    public Command send(Object... formatting) {
        Notification modifiedNotification = notification;
        try {
            modifiedNotification.setDescription(notification.getDescription().formatted(formatting));
        } catch(Error e) {
            modifiedNotification.setLevel(NotificationLevel.ERROR);
            modifiedNotification.setDescription("Malformed formatting");
        }
        return new InstantCommand(() -> Elastic.sendNotification(modifiedNotification));
    }
}
