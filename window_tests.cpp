#include "mainwindow.h"
#include "mavlinkmanager.h"
#include "telemetrymodel.h"
#include "uavsimulator.h"
#include <QApplication>
#include <QCheckBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFile>
#include <QLabel>
#include <QLibraryInfo>
#include <QLocale>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QSignalSpy>
#include <QSpinBox>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QTest>
#include <QTimer>
#include <QTranslator>
#include <QtCharts/QChartView>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DRender/QFrameGraphNode>
#include <Qt3DRender/QNoDraw>
#include <Qt3DRender/QRenderCapture>

namespace {
void check(bool condition, const char* message) { if (!condition) qFatal("FAILED: %s", message); }
template<typename T> T* child(MainWindow& window, const char* name)
{
    auto* result = window.findChild<T*>(name);
    check(result, name);
    return result;
}
}
int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    QTranslator translator;
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const QString translations = QLibraryInfo::path(QLibraryInfo::TranslationsPath);
#else
    const QString translations = QLibraryInfo::location(QLibraryInfo::TranslationsPath);
#endif
    if (translator.load(QLocale(QLocale::Russian), "qtbase", "_", translations)) app.installTranslator(&translator);
    MainWindow window;
    child<QCheckBox>(window, "checkSound")->setChecked(false);
    auto click = [&](const char* name) { child<QPushButton>(window, name)->click(); };
    auto label = [&](const char* name) { return child<QLabel>(window, name)->text(); };
    auto enabled = [&](const char* name) { return child<QPushButton>(window, name)->isEnabled(); };
    auto* status = child<QLabel>(window, "labelLinkStatus");
    auto* commandStatus = child<QLabel>(window, "labelCommandStatus");
    auto* eventLog = child<QListWidget>(window, "listEventLog");
    auto* commandLog = child<QListWidget>(window, "listCommandLog");
    auto* table = child<QTableWidget>(window, "tableTelemetry");
    auto* tabs = child<QTabWidget>(window, "viewTabs");
    auto* manager = child<MavlinkManager>(window, "mavlinkManager");
    auto* model = child<TelemetryModel>(window, "telemetryModel");
    auto* chart = child<QChartView>(window, "chartView");
    auto* series = static_cast<QLineSeries*>(chart->chart()->series().first());
    auto logHas = [&](const QString& text) {
        for (int i = 0; i < eventLog->count(); ++i)
            if (eventLog->item(i)->text().contains(text)) return true;
        return false;
    };
    const QString screenshots = qEnvironmentVariable("GROUNDSTATION_SCREENSHOTS");
    auto screenshot = [&](const QString& name) {
        if (screenshots.isEmpty()) return;
        QDir().mkpath(screenshots);
        window.show();
        QApplication::processEvents();
        check(window.grab().save(screenshots + "/" + name + ".png"), "save screenshot");
    };
    // Ответ оператора на модальный диалог подтверждения.
    QString dialogText;
    auto answer = [&](QMessageBox::StandardButton button, const QString& capture = QString()) {
        QTimer::singleShot(0, &window, [&, button, capture] {
            auto* box = qobject_cast<QMessageBox*>(QApplication::activeModalWidget());
            check(box, "confirmation dialog shown");
            dialogText = box->text();
            if (!capture.isEmpty() && !screenshots.isEmpty()) check(box->grab().save(screenshots + "/" + capture + ".png"), "save dialog");
            box->button(button)->click();
        });
    };
    check(QFile::exists(":/assets/drone_sound.mp3") && QFile::exists(":/assets/critical.mp3")
        && !QPixmap(":/assets/drone.jpg").isNull(), "assets embedded");
    // Порты теста не пересекаются с запущенной НСУ (14540) и эмулятором (14551/14552).
    child<QSpinBox>(window, "spinListenPort")->setValue(34540);
    child<QSpinBox>(window, "spinUavPort")->setValue(34551);
    window.resize(1320, 900);

    // 1. НСУ запущена, порт закрыт.
    check(status->text() == "Связь отсутствует" && status->styleSheet().contains("#d43b3b")
        && !manager->isListening(), "1: no link, port closed");
    check(!enabled("btnTakeoff") && !enabled("btnLand") && !enabled("btnStop"), "commands blocked without link");
    screenshot("01_no_link");
    // 2. «Открыть канал».
    click("btnStartLink");
    check(manager->isListening() && label("labelPort") == "UDP-порт 34540 открыт"
        && label("labelErrorStatus") == "Ошибок нет" && !enabled("btnStartLink"), "2: UDP port open, no errors");
    check(status->text() == "Связь отсутствует" && !enabled("btnTakeoff"), "port open without packets is not a link");
    // 3. Эмулятор шлёт телеметрию 10 Гц.
    QElapsedTimer flight;
    flight.start();
    click("btnStartUav");
    check(QTest::qWaitFor([&] { return model->isLinkConnected() && series->count() >= 10; }, 4000), "3: telemetry arrives");
    check(status->text() == "Связь установлена" && status->styleSheet().contains("#27946f")
        && label("labelSender") == "127.0.0.1:34551", "3: link status and sender from datagram");
    check(label("labelLatitude").startsWith("55.75") && label("labelLongitude").startsWith("37.61")
        && label("labelAltitude") == "0.0 м" && label("labelFlightMode") == "STABILIZED", "3: telemetry labels");
    const quint64 received = manager->receivedPackets();
    check(label("labelPackets").startsWith(QString("Принято: %1").arg(received)) && received >= 10, "3: packet counter");
    check(label("labelVehicle").startsWith("sysid=1 · compid=1 · STANDBY") && enabled("btnTakeoff")
        && enabled("btnLand") && enabled("btnStop"), "3: vehicle identified, commands available");
    check(logHas("СВЯЗЬ · Принят пакет от 127.0.0.1:34551") && logHas("MAVLink HEARTBEAT, sysid=1, compid=1")
        && logHas("Смена режима: STABILIZED"), "3: significant events logged");
    const int logSize = eventLog->count();
    QTest::qWait(500);
    check(eventLog->count() == logSize && manager->receivedPackets() > received, "3: no log entry per packet");
    screenshot("03_telemetry");

    // 4. Повреждённый пакет.
    QSignalSpy positions(manager, &MavlinkManager::telemetryReceived);
    click("btnCorruptPacket");
    check(QTest::qWaitFor([&] { return manager->errorPackets() == 1; }, 2000), "4: corrupted packet detected");
    check(label("labelErrorStatus").contains("контрольная сумма") && logHas("Пакет отклонён, данные не применены")
        && label("labelPackets").endsWith("Ошибочных: 1"), "4: error shown and logged");
    check(QTest::qWaitFor([&] { return positions.size() > 3; }, 2000), "4: work continues with next valid packet");
    screenshot("04_corrupted_packet");
    // 5. Пакеты N+1 и N+2 не отправлены.
    const quint64 lostBefore = manager->lostPackets();
    const quint64 receivedBefore = manager->receivedPackets();
    click("btnDropPackets");
    check(QTest::qWaitFor([&] { return manager->receivedPackets() > receivedBefore + 2; }, 2000), "5: stream continues");
    check(manager->lostPackets() == lostBefore + 2 && logHas("Потеря пакетов: 2")
        && label("labelPackets").contains(QString("Потеряно: %1").arg(lostBefore + 2)), "5: lost counter +2");
    screenshot("05_lost_packets");

    // 6–7. Взлёт 80 м с подтверждением оператора и COMMAND_ACK.
    child<QDoubleSpinBox>(window, "spinTakeoffAltitude")->setValue(80);
    QSignalSpy sent(manager, &MavlinkManager::commandSent);
    QSignalSpy results(manager, &MavlinkManager::commandResult);
    answer(QMessageBox::No);
    click("btnTakeoff");
    check(sent.isEmpty() && logHas("Взлёт отменён оператором"), "operator can cancel takeoff");
    answer(QMessageBox::Yes, "06_takeoff_confirm");
    click("btnTakeoff");
    check(dialogText == "Отправить команду взлёта на высоту 80 м?", "6: confirmation text");
    check(sent.size() == 1 && sent.last()[0].toInt() == 22 && sent.last()[1].toDouble() == 80, "6: MAV_CMD_NAV_TAKEOFF sent");
    check(QTest::qWaitFor([&] { return results.size() == 1; }, 2000), "7: COMMAND_ACK received");
    check(commandStatus->text() == "Статус: Команда принята" && commandStatus->styleSheet().contains("#27946f")
        && commandLog->item(0)->text().contains("COMMAND 22: Команда принята")
        && commandLog->item(1)->text().contains("COMMAND 22: отправлена MAV_CMD_NAV_TAKEOFF, высота 80 м"), "7: accepted");
    check(QTest::qWaitFor([&] { return model->altitude() > 10 && label("labelFlightMode") == "GUIDED"; }, 4000),
        "7: altitude grows after ACK");
    check(series->at(series->count() - 1).y() > 10, "7: altitude chart follows telemetry");
    screenshot("07_takeoff_accepted");
    // Отдельный короткий проход с настоящим OpenGL, без offscreen.
    const QString capturePath = qEnvironmentVariable("GROUNDSTATION_3D_CAPTURE");
    if (!capturePath.isEmpty()) {
        tabs->setCurrentIndex(2);
        window.show();
        Qt3DExtras::Qt3DWindow* view = nullptr;
        for (auto* candidate : QApplication::allWindows())
            if (auto* scene = qobject_cast<Qt3DExtras::Qt3DWindow*>(candidate)) view = scene;
        check(view, "3D view exists");
        Qt3DRender::QRenderCapture* capture = nullptr;
        for (auto* node : view->activeFrameGraph()->findChildren<Qt3DRender::QFrameGraphNode*>()) {
            if (!qobject_cast<Qt3DRender::QNoDraw*>(node)
                && node->findChildren<Qt3DRender::QFrameGraphNode*>(QString(), Qt::FindDirectChildrenOnly).isEmpty()) {
                capture = new Qt3DRender::QRenderCapture(node);
                break;
            }
        }
        check(capture, "3D capture branch");
        // Аппарат набирает высоту и облетает дом; след дополняется раз в секунду.
        QTest::qWait(6000);
        click("focusButton");
        QTest::qWait(1000);
        auto* reply = capture->requestCapture();
        check(QTest::qWaitFor([reply] { return reply->isComplete(); }, 10000), "3D framebuffer rendered");
        check(reply->image().save(capturePath), "save 3D frame");
        delete reply;
        delete capture;
        window.close();
        qInfo("3D rendering check passed");
        return 0;
    }
    // 8. COMMAND_ACK не получен за 3 с.
    child<QCheckBox>(window, "checkAckEnabled")->setChecked(false);
    QElapsedTimer waiting;
    waiting.start();
    click("btnStop");
    check(commandStatus->text().contains("ожидание COMMAND_ACK"), "8: waiting state shown");
    check(QTest::qWaitFor([&] { return results.size() == 2; }, 5000) && waiting.elapsed() >= 2900, "8: ACK timeout after 3 s");
    check(commandStatus->text() == "Статус: Подтверждение не получено" && commandStatus->styleSheet().contains("#d43b3b")
        && logHas("COMMAND_ACK не отправлен"), "8: no ACK status");
    child<QCheckBox>(window, "checkAckEnabled")->setChecked(true);
    screenshot("08_no_ack");
    // Отказ сенсора БПЛА: красное состояние и отклонённая команда.
    child<QCheckBox>(window, "checkSensorFailure")->setChecked(true);
    check(QTest::qWaitFor([&] { return label("labelVehicle").contains("CRITICAL"); }, 2500)
        && logHas("Отказ БПЛА или сенсора: состояние CRITICAL"), "sensor failure shown");
    answer(QMessageBox::Yes);
    click("btnTakeoff");
    check(QTest::qWaitFor([&] { return results.size() == 3; }, 2000)
        && commandStatus->text() == "Статус: Команда отклонена", "takeoff denied during failure");
    screenshot("08_sensor_failure");
    child<QCheckBox>(window, "checkSensorFailure")->setChecked(false);
    check(QTest::qWaitFor([&] { return !label("labelVehicle").contains("CRITICAL"); }, 2500), "vehicle recovered");

    // Таблица: не чаще одной строки в секунду.
    const double seconds = flight.elapsed() / 1000.0;
    check(table->rowCount() >= 3 && table->rowCount() <= seconds + 1, "table rate is at most 1 row per second");
    check(table->item(0, 1)->text() == "1" && table->item(0, 2)->text() == "1"
        && table->item(table->rowCount() - 1, 6)->text() == "GUIDED", "table columns");
    tabs->setCurrentIndex(1);
    screenshot("11_telemetry_table");
    tabs->setCurrentIndex(0);

    // 9. «Отключить БПЛА» — через 3 с связь потеряна, команды заблокированы.
    QElapsedTimer silence;
    silence.start();
    click("btnDisconnectUav");
    check(QTest::qWaitFor([&] { return status->text() == "Связь потеряна"; }, 5000) && silence.elapsed() >= 2900,
        "9: link lost after 3 s");
    check(!enabled("btnTakeoff") && !enabled("btnLand") && !enabled("btnStop"), "safe mode blocks commands");
    check(label("labelLastPacket") != "—" && logHas("Связь с БПЛА потеряна")
        && window.statusBar()->currentMessage().contains("Связь потеряна"), "9: last packet time, log, notice");
    screenshot("09_link_lost");
    // 10. Повторный запуск эмулятора.
    click("btnStartUav");
    check(QTest::qWaitFor([&] { return status->text() == "Связь установлена"; }, 3000), "10: link restored");
    check(enabled("btnTakeoff") && enabled("btnLand") && enabled("btnStop") && logHas("Связь восстановлена")
        && window.statusBar()->currentMessage() == "Связь восстановлена", "10: commands available again");
    check(QTest::qWaitFor([&] { return label("labelVehicle").startsWith("sysid=1"); }, 2000), "10: vehicle known again");
    screenshot("10_link_restored");

    // Ограничение истории графика: серия переиспользуется, точек не больше 300.
    const auto* firstSeries = series;
    for (int i = 0; i < 400; ++i) model->updateTelemetry(55.75222, 37.61556, i % 50, "GUIDED");
    check(series == chart->chart()->series().first() && firstSeries == series
        && series->count() == MainWindow::chartPointLimit, "chart history limited to 300 points");
    check(table->rowCount() <= MainWindow::tableRowLimit, "table history limited");
    // Закрытие канала.
    click("btnStopLink");
    check(!manager->isListening() && status->text() == "Связь отсутствует" && !enabled("btnTakeoff")
        && label("labelPort") == "UDP-порт закрыт", "port closed");
    window.close();
    qInfo("All window checks passed");
}
