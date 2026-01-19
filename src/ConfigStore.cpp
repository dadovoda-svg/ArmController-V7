#include "ConfigStore.h"

static const char* VERSION_KEY = "_cfg_ver";  // chiave interna per la versione

ConfigStore::ConfigStore(const char* nvsNamespace,
                         ConfigParam* params,
                         size_t paramCount,
                         uint32_t version)
: _ns(nvsNamespace),
  _params(params),
  _count(paramCount),
  _version(version),
  _readOnly(false),
  _changeCb(nullptr),
  _changeCbUserData(nullptr)
{
}

bool ConfigStore::begin(bool readOnly)
{
    _readOnly = readOnly;

    if (!_prefs.begin(_ns, _readOnly)) {
        // impossibile aprire NVS
        return false;
    }

    // Legge la versione attuale (0 se non esiste)
    uint32_t storedVersion = _prefs.getUInt(VERSION_KEY, 0);

    if (storedVersion != _version) {
        // Prima esecuzione o struttura cambiata:
        // resetta tutto ai default e salva una nuova versione
        resetToDefaults();
        if (!_readOnly) {
            _prefs.putUInt(VERSION_KEY, _version);
        }
    } else {
        // Versione OK: carica i parametri esistenti
        loadFromNVS();
    }

    return true;
}

void ConfigStore::end()
{
    _prefs.end();
}

ConfigParam* ConfigStore::findParam(const char* key)
{
    if (!key) return nullptr;

    for (size_t i = 0; i < _count; ++i) {
        if (strcmp(_params[i].key, key) == 0) {
            return &_params[i];
        }
    }
    return nullptr;
}

void ConfigStore::loadFromNVS()
{
    for (size_t i = 0; i < _count; ++i) {
        const char* k = _params[i].key;

        // Legge da NVS, se non presente usa default
        float stored = _prefs.getFloat(k, _params[i].defaultValue);
        _params[i].value = stored;

        // Se la chiave non esisteva, getFloat() ha usato il default.
        // Possiamo forzare il salvataggio per garantirne la presenza.
        if (!_readOnly) {
            _prefs.putFloat(k, stored);
        }
    }
}

void ConfigStore::saveAllToNVS()
{
    if (_readOnly) return;

    for (size_t i = 0; i < _count; ++i) {
        _prefs.putFloat(_params[i].key, _params[i].value);
    }
    _prefs.putUInt(VERSION_KEY, _version);
}

float ConfigStore::get(const char* key)
{
    ConfigParam* p = findParam(key);
    if (!p) {
        // Chiave sconosciuta: ritorno 0.0 come fallback
        return 0.0f;
    }
    return p->value;
}

bool ConfigStore::set(const char* key, float newValue)
{
    ConfigParam* p = findParam(key);
    if (!p) {
        return false;
    }

    float oldValue = p->value;

    if (newValue == oldValue) {
        // Nessuna variazione reale, non serve scrivere né chiamare callback
        return true;
    }

    p->value = newValue;

    if (!_readOnly) {
        _prefs.putFloat(p->key, p->value);
    }

    if (_changeCb) {
        _changeCb(_changeCbUserData, key, oldValue, newValue);
    }

    return true;
}

void ConfigStore::resetToDefaults()
{
    for (size_t i = 0; i < _count; ++i) {
        ConfigParam &p = _params[i];
        float oldValue = p.value;
        float newValue = p.defaultValue;

        if (oldValue != newValue) {
            p.value = newValue;

            if (_changeCb) {
                _changeCb(_changeCbUserData, p.key, oldValue, newValue);
            }
        } else {
            p.value = newValue;
        }
    }

    if (!_readOnly) {
        saveAllToNVS();
    }
}

void ConfigStore::setChangeCallback(ConfigChangeCallback cb, void* userData)
{
    _changeCb = cb;
    _changeCbUserData = userData;
}
