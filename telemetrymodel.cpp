#include "telemetrymodel.h"
#include <cmath>

TelemetryModel::TelemetryModel(QObject* parent) : QObject(parent) {}
double TelemetryModel::latitude() const { return m_latitude; }
double TelemetryModel::longitude() const { return m_longitude; }
double TelemetryModel::altitude() const { return m_altitude; }
QString TelemetryModel::flightMode() const { return m_flightMode; }
bool TelemetryModel::isLinkConnected() const { return m_linkConnected; }
QDateTime TelemetryModel::lastUpdate() const { return m_lastUpdate; }
QString TelemetryModel::lastAddress() const { return m_lastAddress; }
quint16 TelemetryModel::lastPort() const { return m_lastPort; }
int TelemetryModel::systemId() const { return m_systemId; }
int TelemetryModel::componentId() const { return m_componentId; }
void TelemetryModel::updateTelemetry(double latitude, double longitude, double altitude, const QString& flightMode)
{
    // Некорректная точка не перезаписывает последнее известное состояние.
    if (!std::isfinite(latitude) || !std::isfinite(longitude) || !std::isfinite(altitude)
        || std::abs(latitude) > 90 || std::abs(longitude) > 180) return;
    if (!m_hasHome) {
        m_hasHome = true;
        m_homeLatitude = latitude;
        m_homeLongitude = longitude;
    }
    m_latitude = latitude;
    m_longitude = longitude;
    m_altitude = altitude;
    m_lastUpdate = QDateTime::currentDateTime();
    if (m_flightMode != flightMode) {
        m_flightMode = flightMode;
        emit flightModeChanged(m_flightMode);
    }
    emit telemetryChanged(m_latitude, m_longitude, m_altitude, m_flightMode);
}
void TelemetryModel::updateLink(bool connected, const QString& address, quint16 port)
{
    // Сигнал только при изменении: журнал не должен получать событие на каждый пакет.
    if (m_linkConnected == connected && (!connected || (m_lastAddress == address && m_lastPort == port))) return;
    m_linkConnected = connected;
    if (!address.isEmpty()) {
        m_lastAddress = address;
        m_lastPort = port;
    }
    emit linkStateChanged(m_linkConnected, address, port);
}
void TelemetryModel::updateVehicle(int systemId, int componentId)
{
    m_systemId = systemId;
    m_componentId = componentId;
}
Point3D TelemetryModel::localPosition() const
{
    if (!m_hasHome) return {};
    // Локальная плоская аппроксимация: достаточно для площадки в сотни метров.
    constexpr double metersPerDegree = 111320.0;
    constexpr double pi = 3.141592653589793;
    const double north = (m_latitude - m_homeLatitude) * metersPerDegree;
    const double east = (m_longitude - m_homeLongitude) * metersPerDegree * std::cos(m_homeLatitude * pi / 180.0);
    return {east, north, m_altitude};
}
