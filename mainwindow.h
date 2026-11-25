#pragma once

#include <QDateTime>
#include <QMainWindow>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
QT_CHARTS_USE_NAMESPACE
#endif

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE
class FlightAudio;
class MavlinkManager;
class Scene3D;
class TelemetryModel;
class UavSimulator;

// Представление НСУ: только виджеты и реакция на сигналы модели и сетевого слоя.
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;
    static constexpr int chartPointLimit = 300;
    static constexpr int tableRowLimit = 200;
private slots:
    void onStartLinkClicked();
    void onStopLinkClicked();
    void onTelemetryChanged(double latitude, double longitude, double altitude, const QString& flightMode);
    void updateChart(double latitude, double longitude, double altitude, const QString& flightMode);
    void updateTable(double latitude, double longitude, double altitude, const QString& flightMode);
    void updateScene(double latitude, double longitude, double altitude, const QString& flightMode);
    void onLinkStateChanged(bool connected, const QString& address, quint16 port);
    void onPacketReceived(quint64 received, quint64 lost);
    void onPacketRejected(quint64 errors, const QString& reason);
    void onMavlinkError(const QString& message);
    void onVehicleDetected(int systemId, int componentId);
    void onVehicleStatusChanged(bool healthy, const QString& status);
    void onFlightModeChanged(const QString& flightMode);
    void onCommandSent(int command, double parameter);
    void onCommandResult(int command, bool accepted, const QString& message);
    void onTakeoffClicked();
    void onLandClicked();
    void onStopClicked();
    void onStartUavClicked();
    void onDisconnectUavClicked();
    void onUavRunningChanged(bool running);
    void onUavCommandReceived(int command, int result);
private:
    bool requireLink(const QString& command);
    void setCommandsEnabled(bool enabled);
    void refreshPackets();
    void refreshVehicle();
    void log(const QString& type, const QString& message, bool error = false);

    Ui::MainWindow* ui;
    TelemetryModel* m_telemetryModel;
    MavlinkManager* m_mavlinkManager;
    UavSimulator* m_uavSimulator;
    FlightAudio* m_audio;
    Scene3D* m_scene;
    QChart* m_chart;
    QLineSeries* m_altitudeSeries;
    QValueAxis* m_timeAxis;
    QValueAxis* m_altitudeAxis;
    QDateTime m_chartStart;
    QDateTime m_lastTableRow;
    QDateTime m_lastRoutePoint;
    quint64 m_reportedLost = 0;
    bool m_linkLost = false;
    bool m_vehicleHealthy = true;
    QString m_vehicleStatus;
};
