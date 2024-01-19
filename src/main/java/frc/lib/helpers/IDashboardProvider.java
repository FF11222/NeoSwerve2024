package frc.lib.helpers;

public interface IDashboardProvider {
    void putDashboard();

    default void registerDashboard() {
        DashboardHelper.register(this);
    }
}
