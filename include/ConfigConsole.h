#pragma once

#include <Arduino.h>
#include "ConfigStore.h"

class ConfigConsole {
public:
    /**
     * @param store Riferimento al ConfigStore da gestire
     * @param io    Stream da usare (es. Serial)
     */
    ConfigConsole(ConfigStore &store, Stream &io);

    /// Stampa banner iniziale / help
    void begin();

    /// Da chiamare frequentemente in loop(): gestisce RX seriale e comandi
    void update();

private:
    enum Mode {
        MODE_NORMAL,
        MODE_IMPORT
    };

    ConfigStore &_store;
    Stream      &_io;
    Mode         _mode;

    static const size_t LINE_BUF_SIZE = 64;
    char   _lineBuf[LINE_BUF_SIZE];
    size_t _lineLen;

    // Nuovo flag per gestire correttamente CR/LF multipli (CRLF, LFCR, ecc.)
    bool   _lastWasTerminator;

    void printPrompt();
    void handleLine(char *line);
    void handleNormalCommand(char *line);
    void handleImportLine(char *line);

    // Comandi
    void cmdHelp();
    void cmdList();
    void cmdGet(const char *key);
    void cmdSet(const char *key, const char *valueStr);
    void cmdExport();
    void cmdImportStart();
    void cmdReset();

    // Helper
    static bool equalsIgnoreCase(const char *a, const char *b);
};
