#include "BQ40Z80.h"

// --- All Data Flash (DF) #defines have been moved to BQ40Z80.h ---

BQ40Z80::BQ40Z80(uint8_t address, TwoWire &wire) {
  _deviceAddress = address;
  _wire = &wire;
}

bool BQ40Z80::begin() {
  _wire->begin();
  _wire->setClock(100000); 
  _wire->beginTransmission(_deviceAddress);
  return (_wire->endTransmission() == 0);
}

// --- STANDARD SBS COMMANDS ---

float BQ40Z80::getVoltage() {
  return readWord(BQ40Z80_CMD_VOLTAGE);
}
float BQ40Z80::getCurrent() {
  return (int16_t)readWord(BQ40Z80_CMD_CURRENT);
}
float BQ40Z80::getAvgCurrent() {
    return (int16_t)readWord(BQ40Z80_CMD_AVG_CURRENT) / 1000.0;
}
float BQ40Z80::getTemperature() {
  // Convert 0.1K reading to Celsius: (Value * 0.1) - 273.15
  return (readWord(BQ40Z80_CMD_TEMPERATURE) * 0.1) - 273.15;
}
uint8_t BQ40Z80::getSOC() { 
  return readByte(BQ40Z80_CMD_SOC);
}
uint8_t BQ40Z80::getSOH() { 
  return readByte(BQ40Z80_CMD_STATE_OF_HEALTH);
}
uint16_t BQ40Z80::getRemCapacity() {
    return readWord(BQ40Z80_CMD_REMAINING_CAPACITY);
}
uint16_t BQ40Z80::getFullCapacity() {
    return readWord(BQ40Z80_CMD_FULL_CHARGE_CAP);
}
uint16_t BQ40Z80::getCycleCount() {
  return readWord(BQ40Z80_CMD_CYCLE_COUNT);
}

/**
 * @brief Reads a specific cell voltage.
 * @param cell_index The cell to read (1-based, e.g., 1 for Cell 1, 7 for Cell 7).
 * @return The cell voltage in mV, or 0 on failure.
 */
uint16_t BQ40Z80::getCellVoltage(uint8_t cell_index) {
  if (cell_index < 1 || cell_index > 7) {
    return 0;
  }

  uint8_t blockBuffer[32]; // Max buffer size needed for DAStatus1

  if (cell_index >= 1 && cell_index <= 4) {
    // DAStatus1 (0x0071) returns 32 bytes: Cell1_V, Cell2_V, Cell3_V, Cell4_V (at start)
    // We only need the first 8 bytes for this, but we'll read what the "working" log did.
    if (!readMACBlock(BQ40Z80_MAC_DASTATUS1, blockBuffer, 8)) {
      return 0;
    }
    int buffer_index = (cell_index - 1) * 2;
    return (uint16_t)blockBuffer[buffer_index] | ((uint16_t)blockBuffer[buffer_index + 1] << 8);
  
  } else { // Cells 5, 6, or 7
    // DAStatus3 (0x007B) returns 18 bytes
    // We only need the first 14 for 7-cell logic.
    if (!readMACBlock(BQ40Z80_MAC_DASTATUS3, blockBuffer, 14)) { 
      return 0;
    }
    // DAStatus3 returns Cell5_V (bytes 0-1), Cell6_V (bytes 6-7), Cell7_V (bytes 12-13)
    if (cell_index == 5) {
      return (uint16_t)blockBuffer[0] | ((uint16_t)blockBuffer[1] << 8);
    } else if (cell_index == 6) {
      return (uint16_t)blockBuffer[6] | ((uint16_t)blockBuffer[7] << 8);
    } else { // cell_index == 7
      return (uint16_t)blockBuffer[12] | ((uint16_t)blockBuffer[13] << 8);
    }
  }
}


// --- ADVANCED MAC COMMANDS ---

uint16_t BQ40Z80::getDeviceType() {
    uint8_t buffer[2]; 
    if (readMACBlock(BQ40Z80_MAC_DEVICE_TYPE, buffer, 2)) {
        return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
    }
    return 0;
}

uint16_t BQ40Z80::getChemID() {
    uint8_t buffer[2]; 
    if (readMACBlock(BQ40Z80_MAC_CHEM_ID, buffer, 2)) {
        return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
    }
    return 0;
}

bool BQ40Z80::getAllCellVoltages(uint16_t* cellVoltages_mV) {
  uint8_t blockBuffer[32]; // Buffer must be 32 bytes for DAStatus1

  // 1. Read Cells 1-4 from DAStatus1 (0x0071) - Expect 8 bytes payload
  if (!readMACBlock(BQ40Z80_MAC_DASTATUS1, blockBuffer, 8)) {
    return false;
  }
  for(int i=0; i<4; i++) {
     cellVoltages_mV[i] = (uint16_t)blockBuffer[i*2] | ((uint16_t)blockBuffer[(i*2)+1] << 8);
  }

  // 2. Read Cells 5-7 from DAStatus3 (0x007B) - Expect 14 bytes payload for 7-cell
  if (!readMACBlock(BQ40Z80_MAC_DASTATUS3, blockBuffer, 14)) { 
    return false;
  }
  cellVoltages_mV[4] = (uint16_t)blockBuffer[0] | ((uint16_t)blockBuffer[1] << 8);  // Cell 5
  cellVoltages_mV[5] = (uint16_t)blockBuffer[6] | ((uint16_t)blockBuffer[7] << 8);  // Cell 6
  cellVoltages_mV[6] = (uint16_t)blockBuffer[12] | ((uint16_t)blockBuffer[13] << 8);// Cell 7

  return true;
}

uint32_t BQ40Z80::getGaugingStatus() {
    uint8_t buffer[4] = {0}; 
    if (!readMACBlock(BQ40Z80_MAC_GAUGING_STATUS, buffer, 3)) { 
      return 0; 
    }
    // GaugingStatus is 3 bytes long
    return (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16);
}

uint32_t BQ40Z80::getChargingStatus() {
    uint8_t buffer[4] = {0}; 
    if (!readMACBlock(BQ40Z80_MAC_CHARGING_STATUS, buffer, 3)) { 
      return 0; 
    }
    // ChargingStatus is 3 bytes long
    return (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16);
}

uint32_t BQ40Z80::getOperationStatus() {
    uint8_t buffer[4] = {0}; 
    if (!readMACBlock(BQ40Z80_MAC_OPERATION_STATUS, buffer, 4)) { 
      return 0; 
    }
    // OperationStatus is 4 bytes long
    return (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24);
}

// --- NEW 32-bit Diagnostic Functions ---

uint32_t BQ40Z80::getSafetyStatus() {
    uint8_t buffer[4] = {0}; 
    if (!readMACBlock(BQ40Z80_MAC_SAFETY_STATUS, buffer, 4)) { 
      return 0; 
    }
    return (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24);
}

uint32_t BQ40Z80::getSafetyAlerts() {
    uint8_t buffer[4] = {0}; 
    if (!readMACBlock(BQ40Z80_MAC_SAFETY_ALERT, buffer, 4)) { 
      return 0; 
    }
    return (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24);
}

uint32_t BQ40Z80::getPFStatus() {
    uint8_t buffer[4] = {0}; 
    if (!readMACBlock(BQ40Z80_MAC_PF_STATUS, buffer, 4)) { 
      return 0; 
    }
    return (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24);
}

/**
 * @brief Decodes a 32-bit SafetyStatus flag into an array of human-readable strings.
 * @param status_flags The 32-bit value from getSafetyStatus().
 * @param fault_buffer A pre-allocated array of Strings to write fault names into.
 * @return The number of active faults found.
 */
int BQ40Z80::decodeSafetyStatus(uint32_t status_flags, String* fault_buffer) {
    int fault_count = 0;
    if (status_flags == 0) return 0; // No faults

    for (int i = 0; i < BQ40Z80_MAX_SAFETY_FLAGS; i++) {
        if (BQ40Z80_SAFETY_FLAG_MASKS[i] == 0) {
            // Check for any other unknown flags
            if (status_flags != 0 && (fault_count < BQ40Z80_MAX_SAFETY_FLAGS)) {
                fault_buffer[fault_count++] = "UNKNOWN (" + String(status_flags, HEX) + ")";
            }
            break; // Stop checking
        }

        if (status_flags & BQ40Z80_SAFETY_FLAG_MASKS[i]) {
            if (fault_count < BQ40Z80_MAX_SAFETY_FLAGS) {
                fault_buffer[fault_count++] = BQ40Z80_SAFETY_FLAG_NAMES[i];
            }
            // Remove this flag from the check
            status_flags &= ~BQ40Z80_SAFETY_FLAG_MASKS[i]; 
        }
    }
    return fault_count;
}

/**
 * @brief Resets the SafetyStatus latches (like CUV) by toggling FET_EN.
 * @return true if the command was sent.
 */
bool BQ40Z80::resetProtectionStatus() {
    // Writing 0x0022 (FET Control) with no data toggles FET_EN, which
    // clears latched safety status flags (like CUV) if the fault is clear.
    return writeWord(BQ40Z80_MAC_MANUFACTURER_ACCESS, BQ40Z80_MAC_FET_CONTROL);
}

// --- LIFETIME DATA FUNCTIONS ---

/**
 * @brief Reads Lifetime Data Block 1 (Max/Min Cell Voltages).
 * @param data A reference to a LifetimeDataBlock1 struct to fill.
 * @return true if the 30-byte read and parsing was successful.
 */
bool BQ40Z80::getLifetimeDataBlock1(LifetimeDataBlock1& data) {
    uint8_t buffer[32]; // Request 32 bytes
    if (!readMACBlock(BQ40Z80_MAC_LIFETIME_BLOCK_1, buffer, 30)) {
        return false;
    }

    // Parse the buffer into the struct
    // 7x Max Cell Voltage (14 bytes)
    for (int i = 0; i < 7; i++) {
        data.cell_max_voltage[i] = (uint16_t)buffer[i*2] | ((uint16_t)buffer[i*2+1] << 8);
    }
    // 7x Min Cell Voltage (14 bytes)
    for (int i = 0; i < 7; i++) {
        data.cell_min_voltage[i] = (uint16_t)buffer[14 + i*2] | ((uint16_t)buffer[14 + i*2+1] << 8);
    }
    // 1x Max Delta Cell Voltage (2 bytes)
    data.max_delta_cell_voltage = (uint16_t)buffer[28] | ((uint16_t)buffer[29] << 8);

    return true;
}

/**
 * @brief Reads Lifetime Data Block 2 (Max Currents, Temps, Resets).
 * @param data A reference to a LifetimeDataBlock2 struct to fill.
 * @return true if the 25-byte read and parsing was successful.
 */
bool BQ40Z80::getLifetimeDataBlock2(LifetimeDataBlock2& data) {
    uint8_t buffer[32]; // Request 32 bytes
    
    if (!readMACBlock(BQ40Z80_MAC_LIFETIME_BLOCK_2, buffer, 25)) {
        return false;
    }

    int idx = 0;
    data.max_charge_current = (int16_t)(buffer[idx] | (buffer[idx+1] << 8));
    idx += 2; // 2
    data.max_discharge_current = (int16_t)(buffer[idx] | (buffer[idx+1] << 8));
    idx += 2; // 4
    data.max_avg_dsg_current = (int16_t)(buffer[idx] | (buffer[idx+1] << 8));
    idx += 2; // 6
    data.max_avg_dsg_power = (int16_t)(buffer[idx] | (buffer[idx+1] << 8));
    idx += 2; // 8
    data.max_temp_cell = (int8_t)buffer[idx++]; // 9
    data.min_temp_cell = (int8_t)buffer[idx++]; // 10
    data.max_delta_cell_temp = (int8_t)buffer[idx++]; // 11
    data.max_temp_int_sensor = (int8_t)buffer[idx++]; // 12
    data.min_temp_int_sensor = (int8_t)buffer[idx++]; // 13
    data.max_temp_fet = (int8_t)buffer[idx++]; // 14
    data.num_shutdowns = buffer[idx++]; // 15
    data.num_partial_resets = buffer[idx++]; // 16
    
    data.num_full_resets = (uint8_t)buffer[idx++]; // 17
    data.num_wdt_resets = (uint8_t)buffer[idx++]; // 18
    
    for (int i = 0; i < 7; i++) {
        data.cb_time_cell[i] = buffer[idx++]; // 19..25
    }

    return true;
}

/**
 * @brief Resets all lifetime data logs to 0.
 * @return true if the command was sent successfully.
 */
bool BQ40Z80::resetLifetimeData() {
    return writeWord(BQ40Z80_MAC_MANUFACTURER_ACCESS, BQ40Z80_MAC_LIFETIME_RESET);
}


// --- CONFIGURATION & SECURITY ---

bool BQ40Z80::unseal() {
  if (!writeWord(BQ40Z80_MAC_MANUFACTURER_ACCESS, BQ40Z80_UNSEAL_KEY1)) return false;
  delay(2);
  if (!writeWord(BQ40Z80_MAC_MANUFACTURER_ACCESS, BQ40Z80_UNSEAL_KEY2)) return false;
  return true;
}

bool BQ40Z80::enterFullAccess() {
  if (!writeWord(BQ40Z80_MAC_MANUFACTURER_ACCESS, BQ40Z80_FULLACCESS_KEY1)) return false;
  delay(2);
  if (!writeWord(BQ40Z80_MAC_MANUFACTURER_ACCESS, BQ40Z80_FULLACCESS_KEY2)) return false;
  return true;
}

bool BQ40Z80::seal() {
  return writeWord(BQ40Z80_MAC_MANUFACTURER_ACCESS, BQ40Z80_MAC_SEAL_DEVICE);
}

/**
 * @brief Checks if the device is SEALED.
 * @return true if sealed, false if unsealed or in full access.
 */
bool BQ40Z80::isSealed() {
    uint32_t op_status = getOperationStatus();
    // Sealed state is SEC1=1, SEC0=1
    bool sealed = (op_status & BQ40Z80_OPSTATUS_SEC1_BIT) && (op_status & BQ40Z80_OPSTATUS_SEC0_BIT);
    return sealed;
}


// --- NEW CONFIGURATION WRAPPER FUNCTIONS ---
bool BQ40Z80::beginConfig() {
  if (!unseal()) {
    return false;
  }
  delay(100);

  if (!enterFullAccess()) {
    return false;
  }
  delay(100);
  return true;
}

bool BQ40Z80::endConfig() {
  if (seal()) {
    return true;
  }
  return false;
}

// --- CONFIGURATION HELPERS: System Interface & Power ---

bool BQ40Z80::setPinConfiguration(uint16_t config_word) {
  return writeDFWord(BQ40Z80_DF_PIN_CONFIG, (int16_t)config_word);
}

bool BQ40Z80::setLEDConfiguration(uint16_t config_word) {
  return writeDFWord(BQ40Z80_DF_LED_CONFIG, (int16_t)config_word);
}

bool BQ40Z80::setSleepConfig(SleepConfig config) {
    bool ok = true;
    ok &= writeDFWord(BQ40Z80_DF_SLEEP_CURRENT, config.sleep_current_mA);
    ok &= writeDFByte(BQ40Z80_DF_VOLTAGE_TIME, config.voltage_time_s);
    ok &= writeDFByte(BQ40Z80_DF_CURRENT_TIME, config.current_time_s);
    return ok;
}


// --- GAUGE-SPECIFIC CONFIGURATION FUNCTIONS: Gauging & Calibration ---

bool BQ40Z80::setUpdateStatus(uint8_t status) {
  // The Update Status field is a single byte at 0x44D8
  return writeDFByte(BQ40Z80_DF_UPDATE_STATUS, status);
}

bool BQ40Z80::setTermVoltage(uint16_t voltage_mV) {
  return writeDFWord(BQ40Z80_DF_TERM_VOLTAGE, (int16_t)voltage_mV);
}

bool BQ40Z80::setChargeTermTaper(uint16_t current_mA) {
  return writeDFWord(BQ40Z80_DF_CHG_TERM_TAPER_CURR, (int16_t)current_mA);
}

bool BQ40Z80::setLoadMode(uint8_t mode) {
  if (mode > 1) {
    return false;
  }
  return writeDFByte(BQ40Z80_DF_LOAD_MODE, mode);
}

/**
 * @brief (NEW) Sets the battery pack as Non-Removable (embedded) or Removable.
 * @param enable true = Non-Removable (ignores PRES pin), false = Removable (uses PRES pin)
 * @return true if the write was successful.
 */
bool BQ40Z80::setNonRemovable(bool enable) {
    int16_t daConfig = 0;
    
    // 1. Read the current DA Configuration word
    if (!readDFWord(BQ40Z80_DF_DA_CONFIG, daConfig)) {
        return false;
    }

    // 2. Modify the [NR] bit (Bit 2)
    if (enable) {
        daConfig |= BQ40Z80_DA_CONFIG_NR_BIT;
    } else {
        daConfig &= ~BQ40Z80_DA_CONFIG_NR_BIT;
    }
    
    // 3. Write the new value back
    return writeDFWord(BQ40Z80_DF_DA_CONFIG, daConfig);
}


// --- CONFIGURATION HELPERS: FET Control ---

bool BQ40Z80::enableAutoFET(bool auto_control) {
  int16_t mfgStatus = 0;
  
  // *** FIX: Read from DF (0x4800) to prevent race condition ***
  if (!readDFWord(BQ40Z80_DF_MFG_STATUS_INIT, mfgStatus)) {
      mfgStatus = 0x0000;
  }
  
  // Set or clear FET_EN bit (Bit 4)
  if (auto_control) {
      mfgStatus |= BQ40Z80_MFG_FET_EN_BIT;  // Set bit (Auto Control / Enabled)
  } else {
      mfgStatus &= ~BQ40Z80_MFG_FET_EN_BIT; // Clear bit (Manual Control / Disabled)
  }
  
  // Write the modified word back to 0x4800
  return writeDFWord(BQ40Z80_DF_MFG_STATUS_INIT, (int16_t)mfgStatus);
}

bool BQ40Z80::setCHGFETToggle() {
    return writeWord(BQ40Z80_MAC_MANUFACTURER_ACCESS, BQ40Z80_MAC_CHGFET_TOGGLE);
}

bool BQ40Z80::setDSGFETToggle() {
    return writeWord(BQ40Z80_MAC_MANUFACTURER_ACCESS, BQ40Z80_MAC_DSGFET_TOGGLE);
}

bool BQ40Z80::resetDevice() {
    // This command forces a full chip reboot
    return writeWord(BQ40Z80_MAC_MANUFACTURER_ACCESS, BQ40Z80_MAC_DEVICE_RESET);
}
// --- GPIO CONTROL FUNCTIONS REMOVED ---


// --- STANDARD PROTECTION CONFIGURATION FUNCTIONS ---

bool BQ40Z80::setDesignCapacity(uint16_t capacity_mAh) {
  return writeDFWord(BQ40Z80_DF_DESIGN_CAP, (int16_t)capacity_mAh);
}

bool BQ40Z80::setCellCount(uint8_t cellCount) {
  if (cellCount < 1 || cellCount > 7) {
    return false;
  }
  
  // The TRM specifies 0x06 for 6-Cell. We directly write the count (1-7).
  uint8_t cellConfig = cellCount; 
  
  return writeDFByte(BQ40Z80_DF_CELL_CONFIG, cellConfig);
}

bool BQ40Z80::setThermistorConfig(bool ts1, bool ts2, bool ts3, bool ts4) {
  // Bit 0: TSInt, Bit 1: TS1, Bit 2: TS2, Bit 3: TS3, Bit 4: TS4
  uint8_t tempEnableConfig = 0;
  if (ts1) tempEnableConfig |= (1 << 1);
  if (ts2) tempEnableConfig |= (1 << 2);
  if (ts3) tempEnableConfig |= (1 << 3);
  if (ts4) tempEnableConfig |= (1 << 4);

  return writeDFByte(BQ40Z80_DF_TEMP_ENABLE, tempEnableConfig);
}

bool BQ40Z80::enableGauging() {
  int16_t mfgStatus = 0;

  // *** FIX: Read from DF (0x4800) to prevent race condition ***
  if (!readDFWord(BQ40Z80_DF_MFG_STATUS_INIT, mfgStatus)) {
      mfgStatus = 0x0000;
  }
  
  // Set GAUGE_EN bit (Bit 3)
  mfgStatus |= BQ40Z80_MFG_GAUGE_EN_BIT; 
  
  return writeDFWord(BQ40Z80_DF_MFG_STATUS_INIT, (int16_t)mfgStatus);
}

bool BQ40Z80::setThermistorCoefficients(int16_t coefficients[9]) {
  uint16_t addresses[9] = {
    BQ40Z80_DF_A1_COEF, BQ40Z80_DF_A2_COEF, BQ40Z80_DF_A3_COEF,
    BQ40Z80_DF_A4_COEF, BQ40Z80_DF_A5_COEF, BQ40Z80_DF_B1_COEF,
    BQ40Z80_DF_B2_COEF, BQ40Z80_DF_B3_COEF, BQ40Z80_DF_B4_COEF
  };

  for (int i = 0; i < 9; i++) {
    if (!writeDFWord(addresses[i], coefficients[i])) {
      return false;
    }
  }
  return true;
}

bool BQ40Z80::setCUVConfig(CUVConfig config) {
  bool ok = true;
  ok &= writeDFWord(BQ40Z80_DF_CUV_THRESH, (int16_t)config.threshold_mV);
  ok &= writeDFByte(BQ40Z80_DF_CUV_DELAY, config.delay_s);
  ok &= writeDFWord(BQ40Z80_DF_CUV_RECOVERY, (int16_t)config.recovery_mV);
  return ok;
}

bool BQ40Z80::setCOVConfig(COVConfig config) {
  bool ok = true;

  // COV stores threshold and recovery values across 5 sequential temperature ranges
  const uint16_t DF_COV_THRESH_STL = 0x4BCE;
  const uint16_t DF_COV_THRESH_STH = 0x4BD0;
  const uint16_t DF_COV_THRESH_HIGH = 0x4BD2;
  const uint16_t DF_COV_THRESH_REC = 0x4BD4;

  const uint16_t DF_COV_RECOVERY_STL = 0x4BD9;
  const uint16_t DF_COV_RECOVERY_STH = 0x4BDB;
  const uint16_t DF_COV_RECOVERY_HIGH = 0x4BDD;
  const uint16_t DF_COV_RECOVERY_REC = 0x4BDF;

  // Write all 5 thresholds (starting at 0x4BCC)
  ok &= writeDFWord(BQ40Z80_DF_COV_THRESH_LOW, (int16_t)config.threshold_mV);
  ok &= writeDFWord(DF_COV_THRESH_STL, (int16_t)config.threshold_mV);
  ok &= writeDFWord(DF_COV_THRESH_STH, (int16_t)config.threshold_mV);
  ok &= writeDFWord(DF_COV_THRESH_HIGH, (int16_t)config.threshold_mV);
  ok &= writeDFWord(DF_COV_THRESH_REC, (int16_t)config.threshold_mV);
  
  // Write delay (single byte shared for all ranges)
  ok &= writeDFByte(BQ40Z80_DF_COV_DELAY, config.delay_s);

  // Write all 5 recoveries (starting at 0x4BD7)
  ok &= writeDFWord(BQ40Z80_DF_COV_RECOVERY_LOW, (int16_t)config.recovery_mV);
  ok &= writeDFWord(DF_COV_RECOVERY_STL, (int16_t)config.recovery_mV);
  ok &= writeDFWord(DF_COV_RECOVERY_STH, (int16_t)config.recovery_mV);
  ok &= writeDFWord(DF_COV_RECOVERY_HIGH, (int16_t)config.recovery_mV);
  ok &= writeDFWord(DF_COV_RECOVERY_REC, (int16_t)config.recovery_mV);
  
  return ok;
}

bool BQ40Z80::setOCC1Config(OCCConfig config) {
  bool ok = true;
  ok &= writeDFWord(BQ40Z80_DF_OCC1_THRESH, (int16_t)config.threshold_mA);
  ok &= writeDFByte(BQ40Z80_DF_OCC1_DELAY, config.delay_s);
  return ok;
}

bool BQ40Z80::setOCD1Config(OCDConfig config) {
  if (config.threshold_mA > 0) {
     return false;
  }
  bool ok = true;
  ok &= writeDFWord(BQ40Z80_DF_OCD1_THRESH, config.threshold_mA);
  ok &= writeDFByte(BQ40Z80_DF_OCD1_DELAY, config.delay_s);
  return ok;
}

// --- NEW BALANCING FUNCTION ---
bool BQ40Z80::setBalancingConfig(BalancingConfig config) {
  uint8_t config_byte = 0;

  if (config.master_enable)      config_byte |= (1 << 0); // CB
  if (config.external_balancing)   config_byte |= (1 << 1); // CBM
  if (config.balance_at_rest)      config_byte |= (1 << 2); // CBR
  if (config.use_weighted_dod_charge) config_byte |= (1 << 3); // CB_CHG_DOD0EW
  if (config.use_weighted_dod_relax)  config_byte |= (1 << 4); // CB_RLX_DOD0EW
  if (config.balance_in_sleep)     config_byte |= (1 << 5); // CBS
  
  return writeDFByte(BQ40Z80_DF_BALANCING_CONFIG, config_byte);
}


// --- Custom AFE Protection Functions (Restored) ---

bool BQ40Z80::setAOLDConfig(AFEConfig config) {
  const uint16_t address = BQ40Z80_DF_AOLD_THRESH;
  // AOLD max index 0x0F (as per TRM Appendix A)
  if (config.threshold_index > 0x0F || config.delay_index > 0x0F) {
      return false;
  }

  // Encoding: [Delay Index (7:4) | Threshold Index (3:0)]
  uint8_t encoded_value = (config.delay_index << 4) | config.threshold_index;
  
  return this->writeDFByte(address, encoded_value); 
}

bool BQ40Z80::setASCCConfig(AFEConfig config) {
  const uint16_t address = BQ40Z80_DF_ASCC_THRESH;
  // ASCC max threshold index 0x07 (3 bits) (as per TRM Appendix A)
  if (config.threshold_index > 0x07 || config.delay_index > 0x0F) {
      return false;
  }

  uint8_t encoded_value = (config.delay_index << 4) | config.threshold_index;
  
  return this->writeDFByte(address, encoded_value); 
}

bool BQ40Z80::setASCD1Config(AFEConfig config) {
  const uint16_t address = BQ40Z80_DF_ASCD1_THRESH;
  // ASCD1 max threshold index 0x07 (3 bits) (as per TRM Appendix A)
  if (config.threshold_index > 0x07 || config.delay_index > 0x0F) {
      return false;
  }

  uint8_t encoded_value = (config.delay_index << 4) | config.threshold_index;
  
  return this->writeDFByte(address, encoded_value); 
}

bool BQ40Z80::setASCD2Config(AFEConfig config) {
  const uint16_t address = BQ40Z80_DF_ASCD2_THRESH;
  // ASCD2 max threshold index 0x07 (3 bits) (as per TRM Appendix A)
  if (config.threshold_index > 0x07 || config.delay_index > 0x0F) {
      return false;
  }

  uint8_t encoded_value = (config.delay_index << 4) | config.threshold_index;
  
  return this->writeDFByte(address, encoded_value); 
}


// --- Custom Temperature Protection Functions (Restored) ---

// Helper function to convert Celsius to the BQ40Z80's 0.1K format
int16_t convertCTo0p1K(int16_t tempC) {
    // T_0.1K = (T_C * 10) + 2731 (where 2731 is 273.1*10)
    return (tempC * 10) + 2731;
}

bool BQ40Z80::setOTCConfig(TempConfig config) {
    int16_t thresh_K = convertCTo0p1K(config.threshold_C);
    int16_t recov_K = convertCTo0p1K(config.recovery_C);

    bool ok = true;
    ok &= writeDFWord(BQ40Z80_DF_OT_CHARGE_THRESH, thresh_K);
    ok &= writeDFWord(BQ40Z80_DF_OT_CHARGE_RECOV, recov_K);
    return ok;
}

bool BQ40Z80::setOTDConfig(TempConfig config) {
    int16_t thresh_K = convertCTo0p1K(config.threshold_C);
    int16_t recov_K = convertCTo0p1K(config.recovery_C);

    bool ok = true;
    ok &= writeDFWord(BQ40Z80_DF_OT_DSG_THRESH, thresh_K);
    ok &= writeDFWord(BQ40Z80_DF_OT_DSG_RECOV, recov_K);
    return ok;
}

bool BQ40Z80::setUTCConfig(TempConfig config) {
    int16_t thresh_K = convertCTo0p1K(config.threshold_C);
    int16_t recov_K = convertCTo0p1K(config.recovery_C);

    bool ok = true;
    ok &= writeDFWord(BQ40Z80_DF_UT_CHARGE_THRESH, thresh_K);
    ok &= writeDFWord(BQ40Z80_DF_UT_CHARGE_RECOV, recov_K);
    return ok;
}

bool BQ40Z80::setUTDConfig(TempConfig config) {
    int16_t thresh_K = convertCTo0p1K(config.threshold_C);
    int16_t recov_K = convertCTo0p1K(config.recovery_C);

    bool ok = true;
    ok &= writeDFWord(BQ40Z80_DF_UT_DSG_THRESH, thresh_K);
    ok &= writeDFWord(BQ40Z80_DF_UT_DSG_RECOV, recov_K);
    return ok;
}


// --- CONFIGURATION HELPERS ---

bool BQ40Z80::writeDFByte(uint16_t address, uint8_t value) {
  // Accessing private member writeDFBlock via 'this'
  return this->writeDFBlock((unsigned int)address, (unsigned char*)&value, (unsigned char)1);
}

bool BQ40Z80::writeDFWord(uint16_t address, int16_t value) {
  uint8_t buf[2];
  buf[0] = (uint8_t)(value & 0xFF);
  buf[1] = (uint8_t)(value >> 8);
  // Accessing private member writeDFBlock via 'this'
  return this->writeDFBlock((unsigned int)address, (unsigned char*)buf, (unsigned char)2);
}

/**
 * @brief (Helper) Reads a 2-byte signed word from a Data Flash address.
 * @param address The 16-bit DF address (e.g., 0x4800).
 * @param value A reference to store the read value.
 * @return true if the read was successful.
 */
bool BQ40Z80::readDFWord(uint16_t address, int16_t& value) {
    uint8_t buffer[2]; // We expect a 2-byte word back

    // Use the existing, working readMACBlock function.
    // We pass the DF address (e.g., 0x4D03) as the MAC command.
    if (readMACBlock(address, buffer, 2)) {
        value = (int16_t)(buffer[0] | (buffer[1] << 8));
        return true;
    } else {
        value = 0;
        return false;
    }
}

// --- LOW LEVEL I2C (Stability Fixes Implemented) ---

uint8_t BQ40Z80::readByte(uint8_t command) {
  _wire->beginTransmission(_deviceAddress);
  _wire->write(command);
  if (_wire->endTransmission(false) != 0) { 
    return 0; 
  }

  // Added stability delay for SBS reads
  delay(5); 

  uint8_t requested = 1;
  _wire->requestFrom(_deviceAddress, requested);

  if (_wire->available()) {
    uint8_t byte = _wire->read();
    return byte;
  }
  return 0; 
}

uint16_t BQ40Z80::readWord(uint8_t command) {
  _wire->beginTransmission(_deviceAddress);
  _wire->write(command);
  if (_wire->endTransmission(true) != 0) {
    return 0; 
  }

  // Increased stability delay (from 1ms to 5ms) for SBS reads
  delay(5); 

  uint8_t requested = 2;
  uint8_t received = _wire->requestFrom(_deviceAddress, requested);

  if (received >= 2) {
    uint8_t lsb = _wire->read();
    uint8_t msb = _wire->read();
    return (uint16_t)(lsb | (msb << 8));
  }
  return 0;
}

bool BQ40Z80::writeWord(uint8_t command, uint16_t data) {
  _wire->beginTransmission(_deviceAddress);
  _wire->write(command);
  _wire->write((uint8_t)(data & 0xFF));       // LSB
  _wire->write((uint8_t)((data >> 8) & 0xFF)); // MSB
  return (_wire->endTransmission(true) == 0);
}

// In BQ40Z80.cpp, replace the old writeDFBlock
bool BQ40Z80::writeDFBlock(unsigned int address, unsigned char* data, unsigned char len) {
  if (len > 28) return false; // Max 28 bytes of data + 2 address + 1 len byte + 1 cmd byte = 32 byte buffer

  _wire->beginTransmission(_deviceAddress);
  _wire->write(BQ40Z80_MAC_BLOCK_ACCESS); // 0x44
  
  _wire->write(len + 2); // Payload Length = Data Length (len) + Address Length (2)
  
  _wire->write((uint8_t)(address & 0xFF)); // Addr LSB
  _wire->write((uint8_t)(address >> 8));   // Addr MSB
  
  if (_wire->write(data, len) != len) {
    _wire->endTransmission(true);
    return false;
  }
  
  if (_wire->endTransmission(true) != 0) {
    return false;
  }

  // CRITICAL: Wait for the Data Flash write to complete.
  // TRM (sluubt5c) p.17, "Page-erase time" is 40ms.
  delay(500); 
  return true;
}

/**
 * @brief Reads a variable-length block from a MAC command.
 * @note This is different from a standard SBS block read. It involves
 * writing the command to 0x00, then reading the data from 0x23.
 * *** REVERTED to "working" logic based on user log ***
 */
bool BQ40Z80::readMACBlock(uint16_t macCommand, uint8_t* buffer, uint8_t expectedLen) {
   // Max payload is 32 bytes
   if (expectedLen > 32) { 
     return false; 
   }

   for(int attempt = 1; attempt <= 3; attempt++) {
       // 1. Write Command to 0x00 (LITTLE ENDIAN)
       if (!writeWord(BQ40Z80_MAC_MANUFACTURER_ACCESS, macCommand)) {
           delay(100); 
           continue;
       }

       delay(50); 

       // 2. Set Read Pointer to 0x23 (using REPEATED START)
       _wire->beginTransmission(_deviceAddress);
       _wire->write(BQ40Z80_MAC_MANUFACTURER_DATA); 
       if (_wire->endTransmission(false) != 0) { // REPEATED START (false)
           delay(100);
           continue;
       }

       // 3. Request 32 bytes (max possible payload + 1 for length byte)
       // We can't request 'expectedLen + 1' because the chip may send more.
       uint8_t requestLen = 32; 
       
       uint8_t received = _wire->requestFrom(_deviceAddress, requestLen); 
       
       if (received < 1) { // Must get at least the length byte
           delay(100);
           continue;
       }
       
       // 4. Read Length Byte
       uint8_t actualLen = _wire->read();
       received--; // Decrement count for the length byte

       // 5. Check if the chip sent *at least* what we need.
       if (received < expectedLen) {
             while(received > 0) { _wire->read(); received--; } // Clear buffer
             delay(100);
             continue;
       }

       // 6. Read Payload
       for(int i = 0; i < expectedLen; i++) {
           buffer[i] = _wire->read();
           received--;
       }
       
       // 7. Clear any remaining bytes from this I2C transaction
       int bytesToClear = received;
       if (bytesToClear > 0) {
         for(int i = 0; i < bytesToClear; i++) {
           if (_wire->available()) _wire->read();
         }
       }
       
       return true; // Success!
   }

   return false; // Failed all retries
}