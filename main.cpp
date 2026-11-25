#include "mainwindow.h"
#include <QApplication>
#include <QLibraryInfo>
#include <QLocale>
#include <QTranslator>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    // Стандартные кнопки QMessageBox («Да» / «Нет») на русском, если перевод Qt установлен.
    QTranslator translator;
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const QString translations = QLibraryInfo::path(QLibraryInfo::TranslationsPath);
#else
    const QString translations = QLibraryInfo::location(QLibraryInfo::TranslationsPath);
#endif
    if (translator.load(QLocale(QLocale::Russian), "qtbase", "_", translations)) app.installTranslator(&translator);
    MainWindow window;
    window.show();
    return app.exec();
}
