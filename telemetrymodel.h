#pragma once

#include "point3d.h"
#include <QDateTime>
#include <QObject>

// Последнее известное состояние БПЛА. Не знает ни о сокетах, ни о виджетах.
class TelemetryModel : public QObject {
    Q_OBJECT
public:
    explicit TelemetryModel(QObject* parent = nullptr);
    void updateTelemetry(double latitude, double longitude, double altitude, const QString& flightMode);
    void updateLink(bool connected, const QString& address, quint16 port);
    void updateVehicle(int systemId, int componentId);
    double latitude() const;
    double longitude() const;
    double altitude() const;
    QString flightMode() const;
    bool isLinkConnected() const;
    QDateTime lastUpdate() const;
    QString lastAddress() const;
    quint16 lastPort() const;
    int systemId() const;
    int componentId() const;
    // Метры на восток / север / вверх от первой полученной точки (дом).
    Point3D localPosition() const;
signals:
    void telemetryChanged(double latitude, double longitude, double altitude, const QString& flightMode);
    void linkStateChanged(bool connected, const QString& address, quint16 port);
    void flightModeChanged(const QString& flightMode);
private:
    double m_latitude = 0.0;
    double m_longitude = 0.0;
    double m_altitude = 0.0;
    QString m_flightMode = "UNKNOWN";
    bool m_linkConnected = false;
    QString m_lastAddress;
    quint16 m_lastPort = 0;
    QDateTime m_lastUpdate;
    int m_systemId = 0;
    int m_componentId = 0;
    bool m_hasHome = false;
    double m_homeLatitude = 0.0;
    double m_homeLongitude = 0.0;
};
