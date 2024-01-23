package frc.lib.vision;

public enum LimelightEntry {
    LED_MODE(0, 0, 3),
    CAM_MODE(0, 0, 1),
    PIPELINE(0, 0, 9);

    public int _default;
    public int min;
    public int max;
    public String attrName;

    LimelightEntry(int _default, int min, int max) {
        this._default = _default;
        this.min = min;
        this.max = max;
    }
}
