 
#include "SerialCommandParser.h"

SerialCommandParser::SerialCommandParser(HardwareSerial& serial,
                                         uint32_t baud,
                                         char term)
    : _serial(serial),
      _baud(baud),
      _terminator(term),
      _echo(false),
      _buffer(),
      _lineTimeoutMs(0),
      _lastCharTime(0)
{
    _buffer.reserve(MAX_LINE_LENGTH);
}

void SerialCommandParser::begin() {
    _serial.begin(_baud);
}

void SerialCommandParser::setEcho(bool enabled) {
    _echo = enabled;
}

void SerialCommandParser::setLineTimeout(uint32_t timeoutMs) {
    _lineTimeoutMs = timeoutMs;
}

void SerialCommandParser::onSimple(char cmd, SimpleCallback callback) {
    for (auto& h : _simpleHandlers) {
        if (h.cmd == cmd) {
            h.cb = callback;
            return;
        }
    }
    _simpleHandlers.push_back(SimpleHandler{cmd, callback});
}

void SerialCommandParser::onAdvanced(const String& name, AdvancedCallback callback) {
    for (auto& h : _advancedHandlers) {
        if (h.name == name) {
            h.cb = callback;
            return;
        }
    }
    _advancedHandlers.push_back(AdvancedHandler{name, callback});
}

void SerialCommandParser::onString(const String& name, StringCallback callback) {
    for (auto& h : _stringHandlers) {
        if (h.name == name) {
            h.cb = callback;
            return;
        }
    }
    _stringHandlers.push_back(StringHandler{name, callback});
}

bool SerialCommandParser::hasSimpleHandlerFor(char cmd) const {
    for (const auto& h : _simpleHandlers) {
        if (h.cmd == cmd) {
            return true;
        }
    }
    return false;
}

bool SerialCommandParser::looksLikeFloat(const String& s) const {
    String t = s;
    t.trim();
    if (t.length() == 0) {
        return false;
    }

    bool hasDigit = false;
    for (size_t i = 0; i < t.length(); ++i) {
        char c = t.charAt(i);
        if (c >= '0' && c <= '9') {
            hasDigit = true;
        } else if (c == '+' || c == '-' || c == '.' || c == 'e' || c == 'E') {
            // caratteri ammessi in un float
        } else {
            // carattere non valido in un numero
            return false;
        }
    }
    return hasDigit;
}

void SerialCommandParser::update() {
    // Gestione timeout linea
    if (_lineTimeoutMs > 0 && _buffer.length() > 0) {
        unsigned long now = millis();
        if ((uint32_t)(now - _lastCharTime) > _lineTimeoutMs) {
            // timeout: scarto il buffer corrente
            _buffer = "";
            _serial.println(F("ERR: line timeout, buffer cleared"));
        }
    }

    while (_serial.available() > 0) {
        char c = static_cast<char>(_serial.read());
        _lastCharTime = millis();

        // Ignora '\r'
        if (c == '\r') {
            continue;
        }

        if (c == _terminator) {
            if (_echo) {
                _serial.println();
            }
            if (_buffer.length() > 0) {
                processLine(_buffer);
                _buffer = "";
            }
        } else {
            if (_echo) {
                _serial.print(c);
            }
            if (_buffer.length() < MAX_LINE_LENGTH - 1) {
                _buffer += c;
            } else {
                // linea troppo lunga: scarto e resetto
                _buffer = "";
                _serial.println(F("ERR: line too long, buffer cleared"));
            }
        }
    }
}

void SerialCommandParser::processLine(String line) {
    line.trim();
    if (line.length() == 0) {
        return;
    }

    int colonIndex = line.indexOf(':');

    if (colonIndex >= 0) {
        // Comando avanzato: "NAME:param1,param2,..."
        String namePart  = line.substring(0, colonIndex);
        String paramPart = line.substring(colonIndex + 1);
        namePart.trim();
        paramPart.trim();
        if (namePart.length() > 0) {
            handleAdvanced(namePart, paramPart);
        } else {
            _serial.print(F("ERR: invalid advanced cmd '"));
            _serial.print(line);
            _serial.println('\'');
        }
        return;
    }

    // Nessun ':', può essere:
    // - comando semplice lettera+float
    // - comando stringa senza parametri

    if (line.length() >= 2) {
        char cmd = line.charAt(0);
        String valueStr = line.substring(1);
        // Se esiste un handler per 'cmd' ed il resto "sembra" un float → comando semplice
        if (hasSimpleHandlerFor(cmd) && looksLikeFloat(valueStr)) {
            handleSimple(line);
            return;
        }
    }
    // Altrimenti lo trattiamo come comando stringa
    handleString(line);
}

void SerialCommandParser::handleSimple(const String& line) {
    if (line.length() < 2) {
        _serial.print(F("ERR: invalid simple cmd '"));
        _serial.print(line);
        _serial.println('\'');
        return;
    }

    char cmd = line.charAt(0);
    String valueStr = line.substring(1);
    valueStr.trim();

    if (!looksLikeFloat(valueStr)) {
        _serial.print(F("ERR: invalid numeric parameter in simple cmd '"));
        _serial.print(line);
        _serial.println('\'');
        return;
    }

    float value = valueStr.toFloat();

    for (auto& h : _simpleHandlers) {
        if (h.cmd == cmd && h.cb) {
            h.cb(value);
            return;
        }
    }

    _serial.print(F("ERR: unknown simple cmd '"));
    _serial.print(cmd);
    _serial.println('\'');
}

void SerialCommandParser::handleAdvanced(const String& namePart,
                                         const String& paramPart) {
    std::vector<float> params;

    if (paramPart.length() > 0) {
        int start = 0;
        while (start < paramPart.length()) {
            int commaIndex = paramPart.indexOf(',', start);
            String token;
            if (commaIndex < 0) {
                token = paramPart.substring(start);
                start = paramPart.length();
            } else {
                token = paramPart.substring(start, commaIndex);
                start = commaIndex + 1;
            }
            token.trim();
            if (token.length() > 0) {
                float val = token.toFloat();
                params.push_back(val);
            }
        }
    }

    for (auto& h : _advancedHandlers) {
        if (h.name == namePart && h.cb) {
            h.cb(params);
            return;
        }
    }

    _serial.print(F("ERR: unknown advanced cmd '"));
    _serial.print(namePart);
    _serial.println('\'');
}

void SerialCommandParser::handleString(const String& line) {
    for (auto& h : _stringHandlers) {
        if (h.name == line && h.cb) {
            h.cb();
            return;
        }
    }

    _serial.print(F("ERR: unknown string cmd '"));
    _serial.print(line);
    _serial.println('\'');
}
