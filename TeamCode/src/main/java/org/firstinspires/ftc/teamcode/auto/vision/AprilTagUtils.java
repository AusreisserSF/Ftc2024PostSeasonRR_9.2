package org.firstinspires.ftc.teamcode.auto.vision;

import java.util.EnumSet;
import java.util.Optional;

public class AprilTagUtils {

    private static final String TAG = AprilTagUtils.class.getSimpleName();

    //## Even though the AprilTag ids below are specific to the CenterStage
    // game, retain this enum in the Core project as a model.

    // AprilTag identifiers
    public enum AprilTagId {
        TAG_ID_1(1), TAG_ID_2(2), TAG_ID_3(3),
        TAG_ID_4(4), TAG_ID_5(5), TAG_ID_6(6),
        TAG_ID_7(7), TAG_ID_8(8), TAG_ID_9(9),
        TAG_ID_10(10);

        private final int numericAprilTagId;

        AprilTagId(int pNumericId) {
            numericAprilTagId = pNumericId;
        }

        public int getNumericId() {
            return numericAprilTagId;
        }

        // Given the numeric id of an AprilTag return its enumeration.
        // See https://stackoverflow.com/questions/27807232/finding-enum-value-with-java-8-stream-api
        public static AprilTagId getEnumValue(int pNumericId) {
            Optional<AprilTagId> matchingTag = EnumSet.allOf(AprilTagId.class).stream()
                    .filter(tag -> tag.getNumericId() == pNumericId)
                    .findFirst();

            return matchingTag.orElseThrow(() -> new RuntimeException(TAG + " Invalid AprilTag number " + pNumericId));
        }
    }

}
