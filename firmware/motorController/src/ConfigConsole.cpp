#include "ConfigConsole.h"
#include <ctype.h>
#include <string.h>

ConfigConsole::ConfigConsole(ConfigStore &store, Stream &io)
: _store(store),
  _io(io),
  _mode(MODE_NORMAL),
  _lineLen(0),
  _lastWasTerminator(false)
{
    _lineBuf[0] = '\0';
}

void ConfigConsole::begin()
{
    _io.println();
    _io.println(F("=== Config Console ==="));
    _io.println(F("Digita 'help' per il menu comandi."));
    printPrompt();
}

void ConfigConsole::printPrompt()
{
    _io.print(F("> "));
}

void ConfigConsole::update()
{
    while (_io.available() > 0) {
        char c = (char)_io.read();
        _io.write(c);

        // Terminatori di riga: gestiamo CR, LF e qualsiasi combinazione (CRLF, LFCR)
        if (c == '\r' || c == '\n') {
            if (!_lastWasTerminator) {
                // Chiudi la riga corrente
                _lineBuf[_lineLen] = '\0';
                handleLine(_lineBuf);

                // Resetta il buffer
                _lineLen = 0;
                _lineBuf[0] = '\0';

                // Prompt per il prossimo comando
                printPrompt();

                // Segna che abbiamo appena gestito un terminatore
                _lastWasTerminator = true;
            }
            // Se _lastWasTerminator è già true, significa che è il secondo
            // carattere di una coppia CRLF o LFCR: lo ignoriamo.
            continue;
        }

        // Qualsiasi carattere non terminatore azzera il flag
        _lastWasTerminator = false;

        // (Opzionale) Gestione backspace (BS=8, DEL=127)
        if (c == 8 || c == 127) {
            if (_lineLen > 0) {
                _lineLen--;
                _lineBuf[_lineLen] = '\0';
                // Se vuoi puoi fare echo del backspace:
                // _io.print("\b \b");
            }
            continue;
        }

        // Accumula nel buffer se c'è spazio
        if (_lineLen < (LINE_BUF_SIZE - 1)) {
            _lineBuf[_lineLen++] = c;
        }
        // Se la linea è troppo lunga, tronchiamo e ignoriamo il resto
    }
}

void ConfigConsole::handleLine(char *line)
{
    // Rimuovi spazi iniziali/finali
    while (*line == ' ' || *line == '\t') line++;
    char *end = line + strlen(line);
    while (end > line && (end[-1] == ' ' || end[-1] == '\t')) {
        *--end = '\0';
    }

    if (line[0] == '\0') {
        // linea vuota, non fare nulla
        return;
    }

    if (_mode == MODE_IMPORT) {
        handleImportLine(line);
    } else {
        handleNormalCommand(line);
    }
}

void ConfigConsole::handleNormalCommand(char *line)
{
    // Usiamo strtok per separare comando e argomenti
    char *cmd  = strtok(line, " ");
    char *arg1 = strtok(NULL, " ");
    char *arg2 = strtok(NULL, ""); // il resto della linea

    if (!cmd) {
        return;
    }

    if (equalsIgnoreCase(cmd, "help") || strcmp(cmd, "?") == 0) {
        cmdHelp();
    } else if (equalsIgnoreCase(cmd, "list")) {
        cmdList();
    } else if (equalsIgnoreCase(cmd, "get")) {
        if (!arg1) {
            _io.println(F("Uso: get <chiave>"));
        } else {
            cmdGet(arg1);
        }
    } else if (equalsIgnoreCase(cmd, "set")) {
        if (!arg1 || !arg2) {
            _io.println(F("Uso: set <chiave> <valore>"));
        } else {
            cmdSet(arg1, arg2);
        }
    } else if (equalsIgnoreCase(cmd, "export")) {
        cmdExport();
    } else if (equalsIgnoreCase(cmd, "import")) {
        cmdImportStart();
    } else if (equalsIgnoreCase(cmd, "reset")) {
        cmdReset();
    } else {
        _io.print(F("Comando sconosciuto: "));
        _io.println(cmd);
        _io.println(F("Digita 'help' per la lista comandi."));
    }
}

void ConfigConsole::handleImportLine(char *line)
{
    // Comandi speciali per chiudere import
    if (equalsIgnoreCase(line, "end") || strcmp(line, ".") == 0) {
        _mode = MODE_NORMAL;
        _io.println(F("Import terminato."));
        return;
    }

    // Formato atteso: chiave=valore
    char *eq = strchr(line, '=');
    if (!eq) {
        _io.print(F("Riga ignorata (manca '='): "));
        _io.println(line);
        return;
    }

    *eq = '\0';
    char *key = line;
    char *valueStr = eq + 1;

    // Trim chiave
    while (*key == ' ' || *key == '\t') key++;
    char *kEnd = key + strlen(key);
    while (kEnd > key && (kEnd[-1] == ' ' || kEnd[-1] == '\t')) {
        *--kEnd = '\0';
    }

    // Trim valore
    while (*valueStr == ' ' || *valueStr == '\t') valueStr++;
    char *vEnd = valueStr + strlen(valueStr);
    while (vEnd > valueStr && (vEnd[-1] == ' ' || vEnd[-1] == '\t')) {
        *--vEnd = '\0';
    }

    if (*key == '\0' || *valueStr == '\0') {
        _io.println(F("Riga ignorata: chiave o valore vuoti."));
        return;
    }

    float value = atof(valueStr);
    if (_store.set(key, value)) {
        _io.print(F("Impostato "));
        _io.print(key);
        _io.print(F(" = "));
        _io.println(value, 6);
    } else {
        _io.print(F("Chiave sconosciuta: "));
        _io.println(key);
    }
}

void ConfigConsole::cmdHelp()
{
    _io.println(F("Comandi disponibili:"));
    _io.println(F("  help / ?         - mostra questo aiuto"));
    _io.println(F("  list             - elenca tutti i parametri"));
    _io.println(F("  get <chiave>     - mostra il valore di un parametro"));
    _io.println(F("  set <chiave> <v> - imposta il valore di un parametro"));
    _io.println(F("  export           - esporta tutti i parametri (chiave=valore)"));
    _io.println(F("  import           - entra in modalita' import (chiave=valore, 'end' per terminare)"));
    _io.println(F("  reset            - resetta tutti i parametri ai valori di default"));
}

void ConfigConsole::cmdList()
{
    _io.println(F("Elenco parametri:"));
    for (size_t i = 0; i < _store.size(); ++i) {
        const ConfigParam &p = _store.at(i);
        _io.print(F("  "));
        _io.print(i);
        _io.print(F(": "));
        _io.print(p.key);
        _io.print(F(" = "));
        _io.print(p.value, 6);
        _io.print(F(" (default="));
        _io.print(p.defaultValue, 6);
        _io.println(F(")"));
    }
}

void ConfigConsole::cmdGet(const char *key)
{
    bool found = false;
    for (size_t i = 0; i < _store.size(); ++i) {
        const ConfigParam &p = _store.at(i);
        if (strcmp(p.key, key) == 0) {
            found = true;
            _io.print(p.key);
            _io.print(F(" = "));
            _io.print(p.value, 6);
            _io.print(F(" (default="));
            _io.print(p.defaultValue, 6);
            _io.println(F(")"));
            break;
        }
    }
    if (!found) {
        _io.print(F("Chiave sconosciuta: "));
        _io.println(key);
    }
}

void ConfigConsole::cmdSet(const char *key, const char *valueStr)
{
    float value = atof(valueStr);
    if (_store.set(key, value)) {
        _io.print(F("Impostato "));
        _io.print(key);
        _io.print(F(" = "));
        _io.println(value, 6);
    } else {
        _io.print(F("Chiave sconosciuta: "));
        _io.println(key);
    }
}

void ConfigConsole::cmdExport()
{
    _io.println(F("# Export configurazione (chiave=valore):"));
    for (size_t i = 0; i < _store.size(); ++i) {
        const ConfigParam &p = _store.at(i);
        _io.print(p.key);
        _io.print('=');
        _io.println(p.value, 6);
    }
    _io.println(F("# Fine export."));
}

void ConfigConsole::cmdImportStart()
{
    _io.println(F("Modalita' import: inserisci righe nella forma chiave=valore."));
    _io.println(F("Digita 'end' o '.' su una riga per terminare."));
    _mode = MODE_IMPORT;
}

void ConfigConsole::cmdReset()
{
    _store.resetToDefaults();
    _io.println(F("Parametri resettati ai valori di default."));
}

bool ConfigConsole::equalsIgnoreCase(const char *a, const char *b)
{
    if (!a || !b) return false;
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) {
            return false;
        }
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}
