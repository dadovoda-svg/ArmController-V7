#pragma once

#include <Arduino.h>
#include <Preferences.h>

// Singolo parametro di configurazione (float)
struct ConfigParam {
    const char* key;       // Nome chiave (max 15 caratteri per NVS)
    float       defaultValue;
    float       value;     // valore attuale in RAM
};

// Callback chiamata quando un parametro cambia valore
typedef void (*ConfigChangeCallback)(void* userData,
                                     const char* key,
                                     float oldValue,
                                     float newValue);

class ConfigStore {
public:
    /**
     * @param nvsNamespace  Namespace NVS (max 15 caratteri, es. "myapp_cfg")
     * @param params        Array di parametri da gestire
     * @param paramCount    Numero di elementi nell'array params
     * @param version       Versione della configurazione (incrementa se cambi layout o default)
     */
    ConfigStore(const char* nvsNamespace,
                ConfigParam* params,
                size_t paramCount,
                uint32_t version = 1);

    /**
     * Apre il namespace NVS e carica i parametri.
     * Se la versione salvata è diversa da quella richiesta, riscrive tutti i default.
     * @param readOnly se true, apre NVS in sola lettura.
     * @return true se ok, false se errore
     */
    bool begin(bool readOnly = false);

    /// Chiude il namespace NVS
    void end();

    /**
     * Restituisce il valore del parametro con quella chiave.
     * Se non viene trovato, ritorna il default relativo e lo scrive in NVS.
     */
    float get(const char* key);

    /**
     * Imposta e salva in NVS il valore del parametro con quella chiave.
     * Se il valore cambia, viene chiamata la callback registrata (se presente).
     * @return true se il parametro esiste ed è stato scritto, false se chiave sconosciuta.
     */
    bool set(const char* key, float newValue);

    /**
     * Resetta tutti i parametri ai default (in RAM e in NVS).
     * Per ogni parametro che cambia, viene chiamata la callback registrata (se presente).
     */
    void resetToDefaults();

    /**
     * Registra una callback chiamata ogni volta che un parametro cambia valore.
     * @param cb        funzione callback (può essere nullptr per disabilitare)
     * @param userData  puntatore utente passato tale e quale alla callback
     */
    void setChangeCallback(ConfigChangeCallback cb, void* userData = nullptr);

    /**
     * Accesso diretto ai parametri per indice (facoltativo).
     */
    size_t size() const { return _count; }
    const ConfigParam& at(size_t idx) const { return _params[idx]; }
    ConfigParam& at(size_t idx) { return _params[idx]; }

private:
    const char*   _ns;
    ConfigParam*  _params;
    size_t        _count;
    uint32_t      _version;
    bool          _readOnly;

    Preferences   _prefs;

    ConfigChangeCallback _changeCb;
    void*                _changeCbUserData;

    ConfigParam* findParam(const char* key);
    void loadFromNVS();
    void saveAllToNVS();
};
