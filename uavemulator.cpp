// Отдельное приложение «Эмулятор БПЛА»: тот же UavSimulator, что встроен в НСУ.
// Запустите НСУ, откройте канал и нажмите «Старт» здесь — пакеты пойдут между процессами.
#include "uavsimulator.h"
#include <QApplication>
#include <QCheckBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QSpinBox>
#include <QTime>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

namespace {
QPushButton* button(const QString& text, QBoxLayout* layout)
{
    auto* result = new QPushButton(text);
    layout->addWidget(result);
    return result;
}
QSpinBox* spin(int minimum, int maximum, int value)
{
    auto* result = new QSpinBox;
    result->setRange(minimum, maximum);
    result->setValue(value);
    return result;
}
}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    QWidget window;
    window.setWindowTitle("Практика №4 — Эмулятор БПЛА · MAVLink 2 / UDP");
    window.resize(460, 520);
    auto* simulator = new UavSimulator(&window);
    auto* layout = new QVBoxLayout(&window);
    layout->addWidget(new QLabel("<b>Эмулятор БПЛА</b> — HEARTBEAT 1 Гц, GLOBAL_POSITION_INT 10 Гц, COMMAND_ACK"));
    auto* form = new QFormLayout;
    auto* address = new QLineEdit("127.0.0.1");
    auto* gcsPort = spin(1, 65535, 14540);
    // 14552: не конфликтует со встроенным эмулятором НСУ (14551).
    auto* localPort = spin(1, 65535, 14552);
    auto* systemId = spin(1, 254, 1);
    form->addRow("Адрес НСУ:", address);
    form->addRow("Порт НСУ:", gcsPort);
    form->addRow("Порт БПЛА:", localPort);
    form->addRow("System ID:", systemId);
    layout->addLayout(form);
    auto* run = new QHBoxLayout;
    auto* start = button("Старт", run);
    auto* stop = button("Отключить БПЛА", run);
    layout->addLayout(run);
    auto* faults = new QHBoxLayout;
    auto* corrupt = button("Повредить пакет", faults);
    auto* drop = button("Пропустить 2 пакета", faults);
    layout->addLayout(faults);
    auto* sensor = new QCheckBox("Отказ сенсора (HEARTBEAT: CRITICAL)");
    auto* ack = new QCheckBox("Отвечать на команды (COMMAND_ACK)");
    ack->setChecked(true);
    layout->addWidget(sensor);
    layout->addWidget(ack);
    auto* state = new QLabel("Остановлен");
    layout->addWidget(state);
    auto* log = new QListWidget;
    layout->addWidget(log, 1);
    stop->setEnabled(false);

    auto write = [log](const QString& message) {
        log->addItem(QString("[%1] %2").arg(QTime::currentTime().toString("hh:mm:ss"), message));
        log->scrollToBottom();
    };
    QObject::connect(start, &QPushButton::clicked, &window, [=] {
        simulator->setLocalPort(static_cast<quint16>(localPort->value()));
        simulator->setSystemId(static_cast<quint8>(systemId->value()));
        simulator->start(address->text(), static_cast<quint16>(gcsPort->value()));
    });
    QObject::connect(stop, &QPushButton::clicked, simulator, &UavSimulator::stop);
    QObject::connect(corrupt, &QPushButton::clicked, &window, [=] {
        simulator->corruptNextPacket();
        write("Следующий пакет будет повреждён");
    });
    QObject::connect(drop, &QPushButton::clicked, &window, [=] {
        simulator->dropNextPackets(2);
        write("Два следующих пакета не будут отправлены");
    });
    QObject::connect(sensor, &QCheckBox::toggled, simulator, &UavSimulator::setSensorFailure);
    QObject::connect(ack, &QCheckBox::toggled, simulator, &UavSimulator::setAckEnabled);
    QObject::connect(simulator, &UavSimulator::runningChanged, &window, [=](bool running) {
        start->setEnabled(!running);
        stop->setEnabled(running);
        for (QWidget* field : {static_cast<QWidget*>(address), static_cast<QWidget*>(gcsPort),
                               static_cast<QWidget*>(localPort), static_cast<QWidget*>(systemId)})
            field->setEnabled(!running);
        write(running ? QString("Старт: порт %1 → %2:%3").arg(simulator->localPort()).arg(address->text()).arg(gcsPort->value())
                      : QString("Остановлен: телеметрия не передаётся"));
    });
    QObject::connect(simulator, &UavSimulator::commandReceived, &window, [=](int command, int result) {
        write(MavlinkMessageParser::commandName(command) + " → "
              + (result < 0 ? QString("COMMAND_ACK не отправлен") : MavlinkMessageParser::resultText(static_cast<quint8>(result))));
    });
    QObject::connect(simulator, &UavSimulator::errorOccurred, &window, write);
    auto* refresh = new QTimer(&window);
    QObject::connect(refresh, &QTimer::timeout, &window, [=] {
        state->setText(QString("%1 · высота %2 м · режим %3")
            .arg(simulator->isRunning() ? "Передаёт" : "Остановлен")
            .arg(simulator->altitude(), 0, 'f', 1).arg(simulator->flightMode()));
    });
    refresh->start(200);
    window.show();
    // --start: сразу начать передачу (удобно для запуска из скрипта).
    if (app.arguments().contains("--start")) start->click();
    return app.exec();
}
