package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

/** Helper methods for working with CTRE Phoenix 6 devices. */
public final class PhoenixUtil {
  private PhoenixUtil() {}

  /**
   * Attempts a Phoenix command multiple times until it succeeds.
   *
   * @param maxAttempts maximum number of attempts before giving up
   * @param command supplier that executes the command and returns its {@link StatusCode}
   */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }
}
