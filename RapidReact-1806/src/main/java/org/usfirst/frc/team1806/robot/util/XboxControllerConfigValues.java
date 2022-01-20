package org.usfirst.frc.team1806.robot.util;

/**
 * A class to hold default values for configuring an Xbox Controller.
 */
public class XboxControllerConfigValues {

    /**
     * The amount that left x must be pushed before any value registers.
     */
    private final Double leftXDeadzone;
    /**
     * The minimum non-0 output for left x. 
     */
    private final Double leftXMinimumOutput;
    /**
     * How linearly to take inputs from left x. 0.0 is fully exponential, 1.0 is fully linear.
     */
    private final Double leftXLinearity;
    /**
     * The amount that left x must be pushed before any value registers.
     */
    private final Double leftYDeadzone;
    /**
     * The minimum non-0 output for left y.
     */
    private final Double leftYMinimumOutput;
    /**
     * How linearly to take inputs from left y. 0.0 is fully exponential, 1.0 is fully linear.
     */
    private final Double leftYLinearity;
    /**
     * The amount that right x must be pushed before any value registers.
     */
    private final Double rightXDeadzone;
    /**
     * The minimum non-0 output for right x.
     */
    private final Double rightXMinimumOutput;
    /**
     * How linearly to take inputs from left x. 0.0 is fully exponential, 1.0 is fully linear.
     */
    private final Double rightXLinearity;
    /**
     * The amount that right y must be pushed before any value registers.
     */
    private final Double rightYDeadzone;
    /**
     * The minimum non-0 output for right y.
     */
    private final Double rightYMinimumOutput;
    /**
     * How linearly to take inputs from right y. 0.0 is fully exponential, 1.0 is fully linear.
     */
    private final Double rightYLinearity;
    /**
     * The amount that the triggers must be pushed before any value registers.
     */
    private final Double triggerDeadzone;
    /**
     * The minimum non-0 output for the triggers.
     */
    private final Double triggerMinimumOutput;
    /**
     * How linearly to take inputs from right y. 0.0 is fully exponential, 1.0 is fully linear.
     */
    private final Double triggerLinearity;
    /**
     * The minimum output to register a trigger as pressed for use as a digital button.
     */
    private final Double triggerAsDigitalDeadzone;

    private XboxControllerConfigValues(final Double leftXDeadzone, final Double leftXMinimumOutput,
            final Double leftXLinearity,
            final Double leftYDeadzone, final Double leftYMinimumOutput, final Double leftYLinearity,
            final Double rightXDeadzone,
            final Double rightXMinimumOutput, final Double rightXLinearity, final Double rightYDeadzone,
            final Double rightYMinimumOutput,
            final Double rightYLinearity, final Double triggerDeadzone, final Double triggerMinimumOutput,
            final Double triggerLinearity,
            final Double triggerAsDigitalDeadzone) {
        this.leftXDeadzone = leftXDeadzone;
        this.leftXMinimumOutput = leftXMinimumOutput;
        this.leftXLinearity = leftXLinearity;
        this.leftYDeadzone = leftYDeadzone;
        this.leftYMinimumOutput = leftYMinimumOutput;
        this.leftYLinearity = leftYLinearity;
        this.rightXDeadzone = rightXDeadzone;
        this.rightXMinimumOutput = rightXMinimumOutput;
        this.rightXLinearity = rightXLinearity;
        this.rightYDeadzone = rightYDeadzone;
        this.rightYMinimumOutput = rightYMinimumOutput;
        this.rightYLinearity = rightYLinearity;
        this.triggerDeadzone = triggerDeadzone;
        this.triggerMinimumOutput = triggerMinimumOutput;
        this.triggerLinearity = triggerLinearity;
        this.triggerAsDigitalDeadzone = triggerAsDigitalDeadzone;
    }

    public Double getLeftXDeadzone() {
        return leftXDeadzone;
    }

    public Double getLeftXMinimumOutput() {
        return leftXMinimumOutput;
    }

    public Double getLeftXLinearity() {
        return leftXLinearity;
    }

    public Double getLeftYDeadzone() {
        return leftYDeadzone;
    }

    public Double getLeftYMinimumOutput() {
        return leftYMinimumOutput;
    }

    public Double getLeftYLinearity() {
        return leftYLinearity;
    }

    public Double getRightXDeadzone() {
        return rightXDeadzone;
    }

    public Double getRightXMinimumOutput() {
        return rightXMinimumOutput;
    }

    public Double getRightXLinearity() {
        return rightXLinearity;
    }

    public Double getRightYDeadzone() {
        return rightYDeadzone;
    }

    public Double getRightYMinimumOutput() {
        return rightYMinimumOutput;
    }

    public Double getRightYLinearity() {
        return rightYLinearity;
    }

    public Double getTriggerDeadzone() {
        return triggerDeadzone;
    }

    public Double getTriggerMinimumOutput() {
        return triggerMinimumOutput;
    }

    public Double getTriggerLinearity() {
        return triggerLinearity;
    }

    public Double getTriggerAsDigitalDeadzone() {
        return triggerAsDigitalDeadzone;
    }

    public static class Builder {
        private boolean isBuilt;

        private Double leftXDeadzone;
        private Double leftXMinimumOutput;
        private Double leftXLinearity;
        private Double leftYDeadzone;
        private Double leftYMinimumOutput;
        private Double leftYLinearity;
        private Double rightXDeadzone;
        private Double rightXMinimumOutput;
        private Double rightXLinearity;
        private Double rightYDeadzone;
        private Double rightYMinimumOutput;
        private Double rightYLinearity;
        private Double triggerDeadzone;
        private Double triggerMinimumOutput;
        private Double triggerLinearity;
        private Double triggerAsDigitalDeadzone;

        public static Builder create() {
            return new Builder();
        }

        private Builder() {
            isBuilt = false;
        }

        private void verifyBuildingState() {
            assert (!isBuilt);
        }

        public XboxControllerConfigValues build() {
            isBuilt = true;
            return new XboxControllerConfigValues(leftXDeadzone, leftXMinimumOutput, leftXLinearity,
                    leftYDeadzone, leftYMinimumOutput, leftYLinearity, rightXDeadzone,
                    rightXMinimumOutput, rightXLinearity, rightYDeadzone, rightYMinimumOutput,
                    rightYLinearity, triggerDeadzone, triggerMinimumOutput, triggerLinearity,
                    triggerAsDigitalDeadzone);
        }

        public Builder withLeftXDeadzone(final Double leftXDeadzone) {
            verifyBuildingState();
            this.leftXDeadzone = leftXDeadzone;
            return this;
        }

        public Builder withLeftXMinimumOutput(final Double leftXMinimumOutput) {
            verifyBuildingState();
            this.leftXMinimumOutput = leftXMinimumOutput;
            return this;
        }

        public Builder withLeftXLinearity(final Double leftXLinearity) {
            verifyBuildingState();
            this.leftXLinearity = leftXLinearity;
            return this;
        }

        public Builder withLeftYDeadzone(final Double leftYDeadzone) {
            verifyBuildingState();
            this.leftYDeadzone = leftYDeadzone;
            return this;
        }

        public Builder withLeftYMinimumOutput(final Double leftYMinimumOutput) {
            verifyBuildingState();
            this.leftYMinimumOutput = leftYMinimumOutput;
            return this;
        }

        public Builder withLeftYLinearity(final Double leftYLinearity) {
            verifyBuildingState();
            this.leftYLinearity = leftYLinearity;
            return this;
        }

        public Builder withRightXDeadzone(final Double rightXDeadzone) {
            verifyBuildingState();
            this.rightXDeadzone = rightXDeadzone;
            return this;
        }

        public Builder withRightXMinimumOutput(final Double rightXMinimumOutput) {
            verifyBuildingState();
            this.rightXMinimumOutput = rightXMinimumOutput;
            return this;
        }

        public Builder withRightXLinearity(final Double rightXLinearity) {
            verifyBuildingState();
            this.rightXLinearity = rightXLinearity;
            return this;
        }

        public Builder withRightYDeadzone(final Double rightYDeadzone) {
            verifyBuildingState();
            this.rightYDeadzone = rightYDeadzone;
            return this;
        }

        public Builder withRightYMinimumOutput(final Double rightYMinimumOutput) {
            verifyBuildingState();
            this.rightYMinimumOutput = rightYMinimumOutput;
            return this;
        }

        public Builder withRightYLinearity(final Double rightYLinearity) {
            verifyBuildingState();
            this.rightYLinearity = rightYLinearity;
            return this;
        }

        public Builder withTriggerDeadzone(final Double triggerDeadzone) {
            verifyBuildingState();
            this.triggerDeadzone = triggerDeadzone;
            return this;
        }

        public Builder withTriggerMinimumOutput(final Double triggerMinimumOutput) {
            verifyBuildingState();
            this.triggerMinimumOutput = triggerMinimumOutput;
            return this;
        }

        public Builder withTriggerLinearity(final Double triggerLinearity) {
            verifyBuildingState();
            this.triggerLinearity = triggerLinearity;
            return this;
        }

        public Builder withTriggerAsDigitalDeadzone(final Double triggerAsDigitalDeadzone) {
            verifyBuildingState();
            this.triggerAsDigitalDeadzone = triggerAsDigitalDeadzone;
            return this;
        }

    }
}
