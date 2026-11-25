#include "mainwindow.h"
#include "flightaudio.h"
#include "mavlinkmanager.h"
#include "scene3d.h"
#include "telemetrymodel.h"
#include "uavsimulator.h"
#include <QHeaderView>
#include <QMessageBox>
#include <QStatusBar>
#include <QTime>
#include <QtCharts/QChartView>
#include <algorithm>
#include "ui_mainwindow.h"

namespace {
const QString uavName = "UAV";
const QString green = "color: #27946f; font-weight: bold;";
const QString amber = "color: #d07618; font-weight: bold;";
const QString red = "color: #d43b3b; font-weight: bold;";
// Площадка Scene3D 600 × 400 м, дом — в её центре.
Point3D scenePoint(const Point3D& local) { return {300 + local.x, 200 + local.y, local.z}; }
}

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_telemetryModel = new TelemetryModel(this);
    m_telemetryModel->setObjectName("telemetryModel");
    m_mavlinkManager = new MavlinkManager(m_telemetryModel, this);
    m_mavlinkManager->setObjectName("mavlinkManager");
    m_uavSimulator = new UavSimulator(this);
    m_uavSimulator->setObjectName("uavSimulator");
    m_audio = new FlightAudio(this);

    m_altitudeSeries = new QLineSeries();
    m_altitudeSeries->setName("Высота, м");
    m_altitudeSeries->setPen(QPen(QColor("#278bc4"), 2));
    m_chart = new QChart();
    m_chart->addSeries(m_altitudeSeries);
    m_chart->setTitle("Высота БПЛА");
    m_chart->legend()->setVisible(true);
    m_chart->legend()->setAlignment(Qt::AlignBottom);
    m_chart->setAnimationOptions(QChart::NoAnimation);
    m_timeAxis = new QValueAxis(m_chart);
    m_timeAxis->setTitleText("Время приёма, с");
    m_timeAxis->setRange(0, 10);
    m_altitudeAxis = new QValueAxis(m_chart);
    m_altitudeAxis->setTitleText("м");
    m_altitudeAxis->setRange(0, 10);
    m_chart->addAxis(m_timeAxis, Qt::AlignBottom);
    m_chart->addAxis(m_altitudeAxis, Qt::AlignLeft);
    m_altitudeSeries->attachAxis(m_timeAxis);
    m_altitudeSeries->attachAxis(m_altitudeAxis);
    ui->chartView->setChart(m_chart);
    ui->chartView->setRenderHint(QPainter::Antialiasing);

    ui->tableTelemetry->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_scene = new Scene3D(ui->sceneHost);
    ui->sceneLayout->addWidget(m_scene);
    m_scene->setDrone(uavName, {}, QColor("#278bc4"));
    m_scene->setSelected(uavName);
    m_scene->setTarget(300, 200, ui->spinTakeoffAltitude->value());
    ui->splitter->setSizes({420, 900});
    ui->rightSplitter->setSizes({560, 240});
    ui->labelLinkStatus->setStyleSheet(red);
    ui->btnStopLink->setEnabled(false);
    ui->btnDisconnectUav->setEnabled(false);
    setCommandsEnabled(false);

    // Модель и сетевой слой связаны сигналами; окно только подписано на результаты.
    connect(m_mavlinkManager, &MavlinkManager::telemetryReceived, m_telemetryModel, &TelemetryModel::updateTelemetry);
    connect(m_telemetryModel, &TelemetryModel::telemetryChanged, this, &MainWindow::onTelemetryChanged);
    connect(m_telemetryModel, &TelemetryModel::telemetryChanged, this, &MainWindow::updateChart);
    connect(m_telemetryModel, &TelemetryModel::telemetryChanged, this, &MainWindow::updateTable);
    connect(m_telemetryModel, &TelemetryModel::telemetryChanged, this, &MainWindow::updateScene);
    connect(m_telemetryModel, &TelemetryModel::flightModeChanged, this, &MainWindow::onFlightModeChanged);
    connect(m_mavlinkManager, &MavlinkManager::linkStateChanged, m_telemetryModel, &TelemetryModel::updateLink);
    connect(m_telemetryModel, &TelemetryModel::linkStateChanged, this, &MainWindow::onLinkStateChanged);
    connect(m_mavlinkManager, &MavlinkManager::vehicleDetected, m_telemetryModel, &TelemetryModel::updateVehicle);
    connect(m_mavlinkManager, &MavlinkManager::vehicleDetected, this, &MainWindow::onVehicleDetected);
    connect(m_mavlinkManager, &MavlinkManager::vehicleStatusChanged, this, &MainWindow::onVehicleStatusChanged);
    connect(m_mavlinkManager, &MavlinkManager::packetReceived, this, &MainWindow::onPacketReceived);
    connect(m_mavlinkManager, &MavlinkManager::packetRejected, this, &MainWindow::onPacketRejected);
    connect(m_mavlinkManager, &MavlinkManager::errorOccurred, this, &MainWindow::onMavlinkError);
    connect(m_mavlinkManager, &MavlinkManager::commandSent, this, &MainWindow::onCommandSent);
    connect(m_mavlinkManager, &MavlinkManager::commandResult, this, &MainWindow::onCommandResult);
    connect(m_uavSimulator, &UavSimulator::runningChanged, this, &MainWindow::onUavRunningChanged);
    connect(m_uavSimulator, &UavSimulator::commandReceived, this, &MainWindow::onUavCommandReceived);
    connect(m_uavSimulator, &UavSimulator::errorOccurred, this, [this](const QString& message) {
        ui->labelUavStatus->setText(message);
        log("ЭМУЛЯТОР", message, true);
    });

    connect(ui->btnStartLink, &QPushButton::clicked, this, &MainWindow::onStartLinkClicked);
    connect(ui->btnStopLink, &QPushButton::clicked, this, &MainWindow::onStopLinkClicked);
    connect(ui->btnTakeoff, &QPushButton::clicked, this, &MainWindow::onTakeoffClicked);
    connect(ui->btnLand, &QPushButton::clicked, this, &MainWindow::onLandClicked);
    connect(ui->btnStop, &QPushButton::clicked, this, &MainWindow::onStopClicked);
    connect(ui->btnStartUav, &QPushButton::clicked, this, &MainWindow::onStartUavClicked);
    connect(ui->btnDisconnectUav, &QPushButton::clicked, this, &MainWindow::onDisconnectUavClicked);
    connect(ui->btnCorruptPacket, &QPushButton::clicked, this, [this] {
        m_uavSimulator->corruptNextPacket();
        log("ЭМУЛЯТОР", "Следующий пакет будет повреждён (байт 3 ^ 0xFF)");
    });
    connect(ui->btnDropPackets, &QPushButton::clicked, this, [this] {
        m_uavSimulator->dropNextPackets(2);
        log("ЭМУЛЯТОР", "Два следующих пакета не будут отправлены");
    });
    connect(ui->checkSensorFailure, &QCheckBox::toggled, m_uavSimulator, &UavSimulator::setSensorFailure);
    connect(ui->checkAckEnabled, &QCheckBox::toggled, m_uavSimulator, &UavSimulator::setAckEnabled);
    connect(ui->checkSound, &QCheckBox::toggled, m_audio, &FlightAudio::set_enabled);
    connect(ui->spinTakeoffAltitude, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double altitude) {
        m_scene->setTarget(300, 200, altitude);
    });
    connect(ui->zoomInButton, &QPushButton::clicked, this, [this] { m_scene->zoom(0.8f); });
    connect(ui->zoomOutButton, &QPushButton::clicked, this, [this] { m_scene->zoom(1.25f); });
    connect(ui->focusButton, &QPushButton::clicked, m_scene, &Scene3D::focusDrone);
    connect(ui->overviewButton, &QPushButton::clicked, m_scene, &Scene3D::resetCamera);
    log("НСУ", "Готова. Откройте канал: НСУ слушает UDP, эмулятор шлёт MAVLink 2");
}

MainWindow::~MainWindow()
{
    m_audio->stop();
    delete ui;
}
void MainWindow::onStartLinkClicked()
{
    const quint16 port = static_cast<quint16>(ui->spinListenPort->value());
    if (!m_mavlinkManager->start(port)) return;
    ui->labelPort->setText(QString("UDP-порт %1 открыт").arg(port));
    ui->labelPort->setStyleSheet(green);
    ui->labelErrorStatus->setText("Ошибок нет");
    ui->labelErrorStatus->setStyleSheet("");
    ui->btnStartLink->setEnabled(false);
    ui->btnStopLink->setEnabled(true);
    ui->spinListenPort->setEnabled(false);
    log("СВЯЗЬ", QString("UDP-порт %1 открыт").arg(port));
}
void MainWindow::onStopLinkClicked()
{
    m_mavlinkManager->stop();
    m_linkLost = false;
    ui->labelPort->setText("UDP-порт закрыт");
    ui->labelPort->setStyleSheet("");
    ui->btnStartLink->setEnabled(true);
    ui->btnStopLink->setEnabled(false);
    ui->spinListenPort->setEnabled(true);
    log("СВЯЗЬ", "UDP-порт закрыт");
}
void MainWindow::onTelemetryChanged(double latitude, double longitude, double altitude, const QString& flightMode)
{
    ui->labelLatitude->setText(QString::number(latitude, 'f', 7));
    ui->labelLongitude->setText(QString::number(longitude, 'f', 7));
    ui->labelAltitude->setText(QString::number(altitude, 'f', 1) + " м");
    ui->labelFlightMode->setText(flightMode);
    if (altitude > 0.5) m_audio->start();
    else m_audio->stop();
}
void MainWindow::updateChart(double, double, double altitude, const QString&)
{
    // Ось времени — реальное время приёма; серия создана один раз и только дополняется.
    const QDateTime received = m_telemetryModel->lastUpdate();
    if (!m_chartStart.isValid()) m_chartStart = received;
    const double time = m_chartStart.msecsTo(received) / 1000.0;
    m_altitudeSeries->append(time, altitude);
    if (m_altitudeSeries->count() > chartPointLimit) m_altitudeSeries->remove(0);
    double high = 10;
    for (const QPointF& point : m_altitudeSeries->points()) high = std::max(high, point.y() * 1.1);
    const double begin = m_altitudeSeries->at(0).x();
    m_timeAxis->setRange(begin, std::max(begin + 10, time));
    m_altitudeAxis->setRange(0, high);
}
void MainWindow::updateTable(double latitude, double longitude, double altitude, const QString& flightMode)
{
    // Не чаще одной строки в секунду и не больше 200 строк.
    const QDateTime received = m_telemetryModel->lastUpdate();
    if (m_lastTableRow.isValid() && m_lastTableRow.msecsTo(received) < 1000) return;
    m_lastTableRow = received;
    const int row = ui->tableTelemetry->rowCount();
    ui->tableTelemetry->insertRow(row);
    const QStringList values{received.toString("hh:mm:ss"), QString::number(m_telemetryModel->systemId()),
        QString::number(m_telemetryModel->componentId()), QString::number(latitude, 'f', 7),
        QString::number(longitude, 'f', 7), QString::number(altitude, 'f', 1), flightMode};
    for (int column = 0; column < values.size(); ++column)
        ui->tableTelemetry->setItem(row, column, new QTableWidgetItem(values[column]));
    if (ui->tableTelemetry->rowCount() > tableRowLimit) ui->tableTelemetry->removeRow(0);
    ui->tableTelemetry->scrollToBottom();
}
void MainWindow::updateScene(double, double, double, const QString&)
{
    const Point3D point = scenePoint(m_telemetryModel->localPosition());
    const QDateTime received = m_telemetryModel->lastUpdate();
    // Маркер движется с каждым пакетом, отрезок следа добавляется раз в секунду.
    if (!m_lastRoutePoint.isValid() || m_lastRoutePoint.msecsTo(received) >= 1000) {
        m_scene->appendRoutePoint(uavName, point);
        m_lastRoutePoint = received;
    } else {
        m_scene->setDronePosition(uavName, point);
    }
}
void MainWindow::onLinkStateChanged(bool connected, const QString& address, quint16 port)
{
    if (connected) {
        ui->labelLinkStatus->setText("Связь установлена");
        ui->labelLinkStatus->setStyleSheet(green);
        ui->labelSender->setText(address + ":" + QString::number(port));
        setCommandsEnabled(true);
        if (m_linkLost) {
            log("СВЯЗЬ", "Связь восстановлена: " + address + ":" + QString::number(port));
            statusBar()->showMessage("Связь восстановлена", 5000);
        } else {
            log("СВЯЗЬ", "Принят пакет от " + address + ":" + QString::number(port));
        }
        m_linkLost = false;
        return;
    }
    // Порт ещё открыт — пакеты перестали приходить; иначе канал закрыт оператором.
    m_linkLost = m_mavlinkManager->isListening();
    ui->labelLinkStatus->setText(m_linkLost ? "Связь потеряна" : "Связь отсутствует");
    ui->labelLinkStatus->setStyleSheet(red);
    ui->labelSender->setText("—");
    setCommandsEnabled(false);
    if (m_linkLost) {
        statusBar()->showMessage("Связь потеряна: команды заблокированы");
        m_audio->alert();
    } else {
        m_audio->stop();
    }
}
void MainWindow::onPacketReceived(quint64, quint64 lost)
{
    refreshPackets();
    ui->labelLastPacket->setText(m_mavlinkManager->lastPacketTime().toString("hh:mm:ss"));
    if (lost > m_reportedLost) {
        log("ПОТЕРИ", QString("Потеря пакетов: %1 (всего %2)").arg(lost - m_reportedLost).arg(lost), true);
        m_reportedLost = lost;
    }
}
void MainWindow::onPacketRejected(quint64, const QString& reason)
{
    refreshPackets();
    ui->labelErrorStatus->setText("Повреждённый пакет: " + reason);
    ui->labelErrorStatus->setStyleSheet(red);
    log("ОШИБКА", "Пакет отклонён, данные не применены: " + reason, true);
}
void MainWindow::onMavlinkError(const QString& message)
{
    ui->labelErrorStatus->setText(message);
    ui->labelErrorStatus->setStyleSheet(red);
    log("ОШИБКА", message, true);
}
void MainWindow::onVehicleDetected(int systemId, int componentId)
{
    log("MAVLINK", QString("MAVLink HEARTBEAT, sysid=%1, compid=%2").arg(systemId).arg(componentId));
    refreshVehicle();
}
void MainWindow::onVehicleStatusChanged(bool healthy, const QString& status)
{
    m_vehicleStatus = status;
    if (!healthy && m_vehicleHealthy) {
        log("АВАРИЯ", "Отказ БПЛА или сенсора: состояние " + status, true);
        statusBar()->showMessage("Авария БПЛА: " + status, 5000);
        m_audio->alert();
    } else if (healthy && !m_vehicleHealthy) {
        log("БПЛА", "Состояние восстановлено: " + status);
    }
    m_vehicleHealthy = healthy;
    refreshVehicle();
}
void MainWindow::onFlightModeChanged(const QString& flightMode)
{
    log("РЕЖИМ", "Смена режима: " + flightMode);
}
void MainWindow::onTakeoffClicked()
{
    if (!requireLink("взлёта")) return;
    // Вариант 3 задания 3: подтверждение оператором.
    const QMessageBox::StandardButton answer = QMessageBox::question(
        this,
        "Подтверждение команды",
        "Отправить команду взлёта на высоту " + QString::number(ui->spinTakeoffAltitude->value(), 'f', 0) + " м?",
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);
    if (answer != QMessageBox::Yes) {
        log("КОМАНДА", "Взлёт отменён оператором");
        return;
    }
    m_mavlinkManager->sendTakeoff(ui->spinTakeoffAltitude->value());
}
void MainWindow::onLandClicked()
{
    if (requireLink("посадки")) m_mavlinkManager->sendLand();
}
void MainWindow::onStopClicked()
{
    if (requireLink("остановки")) m_mavlinkManager->sendHold();
}
bool MainWindow::requireLink(const QString& command)
{
    if (m_telemetryModel->isLinkConnected()) return true;
    QMessageBox::warning(this, "Команда недоступна", "Невозможно отправить команду " + command + ": нет связи с БПЛА.");
    return false;
}
void MainWindow::onCommandSent(int command, double parameter)
{
    const QString name = MavlinkMessageParser::commandName(command);
    const QString detail = command == mavlink::MAV_CMD_NAV_TAKEOFF ? QString(", высота %1 м").arg(parameter) : QString();
    ui->labelCommandStatus->setText("Статус: " + name + " отправлена, ожидание COMMAND_ACK");
    ui->labelCommandStatus->setStyleSheet(amber);
    ui->listCommandLog->insertItem(0, QString("[%1] COMMAND %2: отправлена %3%4")
        .arg(QTime::currentTime().toString("hh:mm:ss")).arg(command).arg(name, detail));
    log("КОМАНДА", "Отправлена " + name + detail);
}
void MainWindow::onCommandResult(int command, bool accepted, const QString& message)
{
    if (accepted) {
        ui->labelCommandStatus->setText("Статус: " + message);
        ui->labelCommandStatus->setStyleSheet(green);
    } else {
        ui->labelCommandStatus->setText("Статус: " + message);
        ui->labelCommandStatus->setStyleSheet(red);
    }
    const QString record = QString("[%1] COMMAND %2: %3")
        .arg(QTime::currentTime().toString("hh:mm:ss"))
        .arg(command)
        .arg(message);
    ui->listCommandLog->insertItem(0, record);
    log("КОМАНДА", MavlinkMessageParser::commandName(command) + ": " + message, !accepted);
}
void MainWindow::onStartUavClicked()
{
    m_uavSimulator->setLocalPort(static_cast<quint16>(ui->spinUavPort->value()));
    m_uavSimulator->start("127.0.0.1", static_cast<quint16>(ui->spinListenPort->value()));
}
void MainWindow::onDisconnectUavClicked()
{
    m_uavSimulator->stop();
}
void MainWindow::onUavRunningChanged(bool running)
{
    ui->btnStartUav->setEnabled(!running);
    ui->btnDisconnectUav->setEnabled(running);
    ui->spinUavPort->setEnabled(!running);
    if (running) {
        const QString text = QString("Эмулятор: порт %1 → 127.0.0.1:%2, 10 Гц")
            .arg(m_uavSimulator->localPort()).arg(ui->spinListenPort->value());
        ui->labelUavStatus->setText(text);
        log("ЭМУЛЯТОР", text);
    } else {
        ui->labelUavStatus->setText("Эмулятор остановлен");
        log("ЭМУЛЯТОР", "БПЛА отключён: телеметрия не передаётся");
    }
}
void MainWindow::onUavCommandReceived(int command, int result)
{
    if (result < 0) log("ЭМУЛЯТОР", MavlinkMessageParser::commandName(command) + " получена, COMMAND_ACK не отправлен");
}
void MainWindow::setCommandsEnabled(bool enabled)
{
    // Безопасный режим: без связи критичные команды недоступны.
    ui->btnTakeoff->setEnabled(enabled);
    ui->btnLand->setEnabled(enabled);
    ui->btnStop->setEnabled(enabled);
}
void MainWindow::refreshPackets()
{
    ui->labelPackets->setText(QString("Принято: %1 | Потеряно: %2 | Ошибочных: %3")
        .arg(m_mavlinkManager->receivedPackets())
        .arg(m_mavlinkManager->lostPackets())
        .arg(m_mavlinkManager->errorPackets()));
}
void MainWindow::refreshVehicle()
{
    ui->labelVehicle->setText(QString("sysid=%1 · compid=%2 · %3").arg(m_telemetryModel->systemId())
        .arg(m_telemetryModel->componentId()).arg(m_vehicleStatus.isEmpty() ? "—" : m_vehicleStatus));
    ui->labelVehicle->setStyleSheet(m_vehicleHealthy ? "" : red);
}
void MainWindow::log(const QString& type, const QString& message, bool error)
{
    auto* item = new QListWidgetItem(QString("[%1] %2 · %3")
        .arg(QTime::currentTime().toString("hh:mm:ss"), type, message), ui->listEventLog);
    if (error) item->setForeground(QColor("#d43b3b"));
    if (ui->listEventLog->count() > 500) delete ui->listEventLog->takeItem(0);
    ui->listEventLog->scrollToBottom();
}
