 
#pragma once

#include <Arduino.h>
#include <vector>
#include <functional>

class SerialCommandParser {
public:
    using SimpleCallback   = std::function<void(float)>;
    using AdvancedCallback = std::function<void(const std::vector<float>&)>;
    using StringCallback   = std::function<void(void)>;

    SerialCommandParser(HardwareSerial& serial = Serial,
                        uint32_t baud = 115200,
                        char term = '\n');

    // Da chiamare in setup()
    void begin();

    // Da chiamare nel loop()
    void update();

    // Comando semplice: 'A123.45'
    void onSimple(char cmd, SimpleCallback callback);

    // Comando avanzato: "SET:1.0,2.0"
    void onAdvanced(const String& name, AdvancedCallback callback);

    // Nuovo: comando stringa senza parametri: "STOP", "RST", "HOME"
    void onString(const String& name, StringCallback callback);

    // Echo dei caratteri letti
    void setEcho(bool enabled);

    // Timeout di linea in millisecondi (0 = disabilitato)
    void setLineTimeout(uint32_t timeoutMs);

private:
    HardwareSerial& _serial;
    uint32_t        _baud;
    char            _terminator;
    bool            _echo;

    String _buffer;

    struct SimpleHandler {
        char           cmd;
        SimpleCallback cb;
    };

    struct AdvancedHandler {
        String          name;
        AdvancedCallback cb;
    };

    struct StringHandler {
        String        name;
        StringCallback cb;
    };

    std::vector<SimpleHandler>   _simpleHandlers;
    std::vector<AdvancedHandler> _advancedHandlers;
    std::vector<StringHandler>   _stringHandlers;

    static const size_t MAX_LINE_LENGTH = 64;

    uint32_t     _lineTimeoutMs;
    unsigned long _lastCharTime;

    void processLine(String line);
    void handleSimple(const String& line);
    void handleAdvanced(const String& namePart, const String& paramPart);
    void handleString(const String& line);

    bool hasSimpleHandlerFor(char cmd) const;
    bool looksLikeFloat(const String& s) const;
};
